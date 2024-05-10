(
Tdef(\readSerial).play;
Tdef(\colorControl).play;
)
(
Window.closeAll;
Tdef(\readSerial).stop;
Tdef(\colorControl).stop;
~port.close;
)


// execute the following code
(
// PRESSURE SENSOR MATRIX ON ARDUINO MEGA

var scale, trig_threshold, // GUI and matrix properties
range_min, range_max,
columns, rows, // user specified
sound_folder_path, // user specified
views, views_full, // handle GUI layouts & views
play_slice, slice_threshold, slice_metric; // inside twoCorpus instrument, but also called at colorCheck rate

sound_folder_path = "/Users/Robin/Desktop/TUTTO/ATTIVI/SillyCode/Samples_esther/";
//sound_folder_path = FluidFilesPath(); // audio samples to map (directory)
scale = 60; // resize matrix window
trig_threshold = 0.1; // thresh detection on matrix (0-1)
range_min = 10; // min sensor value mapped
range_max = 600; // max sensor value mapped
slice_threshold = 0.5; // slice threshold for flucoma analysis (0-1, the bigger the fewer points)
slice_metric = 0; // type of slicing method for FluidBufOnsetSlice
columns = 15; // number of columns (to digital pins)
rows = 13; // number of rows (to analog pins)

Tdef(\readSerial).stop;
Tdef(\colorControl).stop;
Tdef(\colorCheck).stop;
SerialPort.closeAll;

//s.boot;

//Buffer.freeAll(s);

// TO DO: automate port iniz
~port = SerialPort.new("/dev/tty.usbserial-A703Y978", 115200);
//CmdPeriod.doOnce{SerialPort.closeAll; Window.closeAll};

~point = Buffer.alloc(s,2);  // allocates buffer to store coordinates mapped from controller grid

// this task updates values from arduino in ~res, column by column
Tdef(\readSerial, {
	loop{
		var byte, str, res;
		if(~port.read==10,
			{	str = "";
				while(
					{byte = ~port.read; byte!=13},
					{str = str++byte.asAscii}
				);
				res = str.split($ );
				~res=res;
			});
	}
}).play;

// initialise GUI window
Window.closeAll;
w = Window.new(bounds:Rect(800,300,20+(scale*columns),20+(scale*rows))).front; // main window


// FluCoMa audio analysis
~twoD_instrument = {
	arg folder;

	fork{
		var loader = FluidLoadFolder(folder).play(s, {"done".postln});
		var src, analyses, normed, tree;
		var indices = Buffer(s); // will store slice indices

		s.sync;

		//
		if(loader.buffer.numChannels > 1, { // if stereo, compose 2 channels buffers into 1 channel buffer
			src = Buffer(s);
			FluidBufCompose.processBlocking(s, loader.buffer, startChan:0, numChans:1, destination: src, destStartChan:0, gain:-6.dbamp); // left
			FluidBufCompose.processBlocking(s, loader.buffer, startChan:1, numChans:1, destination: src, destStartChan:0, gain:-6.dbamp, destGain: 1); // right
		}, { // if mono, load it as it is
			src = loader.buffer;
		});


		"slicing in process.....".postln;

		FluidBufOnsetSlice.processBlocking(s, src, metric: slice_metric, threshold: slice_threshold, indices: indices, action: {
		//FluidBufAmpSlice.processBlocking(s, src, threshold: slice_threshold, indices: indices, action: {
		FluidNoveltyFeature
			"SLICES CREATED".postln;
			"average seconds per slice: %".format(src.duration / indices.numFrames).postln;

		});
/*
		// function to perform the slices
		play_slice = {
			arg index;
			{
				var startsamp = Index.kr(indices, index);
				var stopsamp = Index.kr(indices, index+1);
				var phs = Phasor.ar(0,BufRateScale.ir(src),startsamp,stopsamp);
				var sig = BufRd.ar(1, src, phs);
				var dursecs = (stopsamp-startsamp) / BufSampleRate.ir(src);
				var env;
				dursecs = min(dursecs,1);
				env = EnvGen.kr(Env([0,1,1,0],[0.03, dursecs-(0.06), 0.03]), doneAction:2);
				sig.dup * env;
			}.play;
		};
*/
		SynthDef.new(\synthSlice, {
			arg index, amp,
			atk=0.1, rel=0.1,
			rate=1;
			var startsamp, stopsamp, phs,
			sig, dursecs, env, out;
			startsamp = Index.kr(indices, index);
			stopsamp = Index.kr(indices, index+1);
			//phs = Phasor.ar(0,BufRateScale.ir(src),startsamp,stopsamp);
			//sig = BufRd.ar(1, src, phs);
			sig = PlayBuf.ar(1, src, rate, 1, startsamp/BufSampleRate.ir(src), 1);
			dursecs = (stopsamp-startsamp) / BufSampleRate.ir(src);
			env = EnvGen.kr(Env([0,1,1,0],[atk, dursecs-(atk+rel), rel]), doneAction:2);
			out = Out.ar(0, sig!2) * env * amp;
		}).add;

		// flucoma dataset to handle analyses results
		analyses = FluidDataSet(s);

		indices.loadToFloatArray(action:{
			arg fa;

			var umapped;
			var mfcc = Buffer(s);
			var stats = Buffer(s);
			var flat = Buffer(s);
			var point = Buffer(s);

			// create slices (start-end of each)
			fa.doAdjacentPairs{
				arg start, end, i;
				var num = end-start; // n. of samples to analyse in each frame

				// MFCC analysis
				FluidBufMFCC.processBlocking(s, src, start, num, features:mfcc, numCoeffs:13, startCoeff:1);
				// stats on analysis
				FluidBufStats.processBlocking(s, mfcc, stats: stats, select: [\mean]);
				// flattened stats
		 		FluidBufFlatten.processBlocking(s, stats, destination:flat);

				analyses.addPoint(i,flat);

				"slice % / %0".format(i, fa.size).postln;
				if((i%100) == 99, {  // synch every 100 slices
					s.sync;
				});
			};

			analyses.print;

			// declare another dataset to store the UMAP dimension reduction results
			umapped = FluidDataSet(s);
			// UMAP dimensionality reduction 813->2)
			FluidUMAP(s, numNeighbours:15, minDist:0.5).fitTransform(analyses, umapped, action: {
				"UMAP done.".postln;
			});


			// normalise UMAPped dataset
			normed = FluidDataSet(s);
			FluidNormalize(s).fitTransform(umapped, normed);

			// fit a k-mean decision tree on normalised params (coordinates)
			~tree = FluidKDTree(s);
			~tree.fit(normed);

			// dump normalised dataset (array) to put it on the plot
			normed.dump({
				arg dict;
				var point = Buffer.alloc(s,2);
				var previous = nil;
				dict.postln;
				{FluidPlotter(w, w.view.bounds+5, dict, standalone:false , mouseMoveAction:{
					arg view, x, y;
					[x,y].postln;
					point.setn(0,[x,y]);

					// current mouse position to nearest neighbor (use whatever "point" coming from something)
					~tree.kNearest(point, 1, {
						arg nearest;
						if(nearest != previous, {
							nearest.postln;
							view.highlight_(nearest);
							play_slice.(nearest.asInteger);
							previous=nearest;
						});
					});
				}).background_(Color.grey(0.5,0))}.defer;
			});

		});
	};
};

// call the whole Flucoma process
~twoD_instrument.(sound_folder_path);

// creates a series of columns (background black)
views = ~res[0..14].collect{
	arg m, ind;
	CompositeView(w, Rect(10+(ind*scale), 10, scale, scale*rows))
	.background_(Color.gray(0))
};


// populate views (columns) full with racks of boxes
views_full = views.collect{
	arg o, index;
	rows.collect{
		arg p;
		CompositeView(views[index], Rect(0,0+(p*scale), scale,scale))
		.background_(Color.gray(0.5)) // <-- this is the value to modulate
	}
};
// this task updates the GUI at 20ms rate (slower than arduino readings)
Tdef(\colorControl, {
	loop{

		var coor_coll = [];
		~coor_coll = 0;

		views_full.do{
			arg view, index;
			var rowStart, rowEnd,
			shades, store_index, scope,
			coordinates, coor_coll;

			// fragments ~res in as many chunks as rows, to place it in views
			rowStart = index*rows;
			rowEnd = (index*rows)+(rows-1);
			// raw values coming from arduino, in the current chunk
			scope = ~res[rowStart..rowEnd];

			// map raw values with color intensity
			shades = scope.collect{
				arg j;
				j.asFloat.linlin(range_min,range_max,1,0);
			};

			// store index to use in nested scopes
			store_index = index;
			// check for points exceeding specified threshold
			coordinates = scope.collect{
				arg k, idx;

				if(k.asFloat.linlin(range_min,range_max,1,0) > trig_threshold, {
					var arr =
					[ // if the normalised value exceeds trig_threshold, add coordinates too coor_coll
						store_index.asFloat.linlin(0,columns-1,0,1), // x
						idx.asFloat.linlin(0,rows-1,1,0), // y
						k.asFloat.linlin(range_min,range_max,1,0) // normalised value
					];
					coor_coll = coor_coll.add(arr);
					~coor_coll = coor_coll; // make it global
				});
				// this value will go in coordinates
				k.asFloat.linlin(range_min,range_max,1,0);
			};

			// update grid GUI shades
			views_full[index].do{
				arg currentView, counter;
				{ currentView.background_(Color.gray(shades[counter])) }.defer;
			};

		};

		0.02.wait;
		}
}).play(AppClock);

// update view action value (check the color and if above threshold, collect coordinates


~previous = nil; // introduce the variable

Tdef(\colorCheck, {//observes grid status, plays slices at need
	loop{
		var neighbors; // num of neighbors
		views_full.do{
			arg view, index;
			var pair, value;
			pair = ~coor_coll.collect{
				arg item, i;
				[item[0], item[1]]
			};
			~point.setn(0,pair.flatten);

			~value = ~coor_coll.collect{
				arg item, i;
				item[2]; // this will go into amplitudes
			};


		};

		if(~coor_coll.size != 0, { // only if non-empty list of coordinates perform knearest search
			neighbors = ~coor_coll.size;

			~tree.kNearest(~point, neighbors, action://not 1 but as many nei.ghbours as current gridpoints detected
				{	arg near; // pass array instead of 1 nearest
					if(near != ~previous, {
						near.postln;
						near.do{
							arg i, count;
							//play_slice.(near.asInteger, amp, atk, rel)
							//play_slice.(near.asInteger, ~value[count].asFloat);
							Synth(\synthSlice, [
								\index, i.asInteger,
								\amp, ~value[count].asFloat.linlin(0,1,0,0.7),

							]);
						};
						~previous=near;
					});
			});
		});

		0.1.wait;
	}
}).play;

)
