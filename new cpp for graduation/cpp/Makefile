FLAG =-L/usr/local/lib -lopencv_aruco -lpthread -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_dpm -lopencv_freetype -lopencv_fuzzy -lopencv_line_descriptor -lopencv_optflow -lopencv_reg -lopencv_saliency -lopencv_stereo -lopencv_stitching -lopencv_structured_light -lopencv_phase_unwrapping -lopencv_rgbd -lopencv_superres -lopencv_surface_matching -lopencv_tracking -lopencv_datasets -lopencv_text -lopencv_face -lopencv_plot -lopencv_dnn -lopencv_videostab -lopencv_xfeatures2d -lopencv_shape -lopencv_video -lopencv_ximgproc -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_xobjdetect -lopencv_objdetect -lopencv_ml -lopencv_xphoto -lopencv_highgui -lopencv_videoio -lopencv_imgcodecs -lopencv_photo -lopencv_imgproc -lopencv_core -ljpeg -lpng -lz -ltiff -ltesseract -llept -lgtk-x11-2.0 -lgdk-x11-2.0 -lpangocairo-1.0 -latk-1.0 -lcairo -lgdk_pixbuf-2.0 -lgio-2.0 -lpangoft2-1.0 -lpango-1.0 -lgobject-2.0 -lfribidi -lfontconfig -lfreetype -lgthread-2.0 -lglib-2.0 -ldl -lm -lpthread -lrt

edit: main.o uart.o order.o cmd_queue.o cut_img.o posture_test.o enhancement.o sgbm.o
	g++ $(FLAG) -o edit main.o order.o uart.o cmd_queue.o cut_img.o posture_test.o enhancement.o sgbm.o

main.o:main.cpp order.h uart.hpp cmd_queue.h sgbm.h
	g++ -c main.cpp 

uart.o:uart.cpp uart.hpp cmd_queue.h
	g++ -c uart.cpp 

cmd_queue.o:cmd_queue.c cmd_queue.h
	g++ -c cmd_queue.c

order.o:order.cpp order.h uart.hpp opencv_hpp.h cut_img.h 
	g++ -c order.cpp

cut_img.o:cut_img.cpp cut_img.h
	g++ -c cut_img.cpp 

posture_test.o:posture_test.cpp posture_test.h
	g++ -c posture_test.cpp

enhancement.o: enhancement.cpp enhancement.h
	g++ -c enhancement.cpp

sgbm.o: sgbm.cpp sgbm.h
	g++ -c sgbm.cpp

clean:
	rm -f edit.o main.o uart.o order.o cmd_queue.o cut_img.o posture_test.o enhancement.o sgbm.o