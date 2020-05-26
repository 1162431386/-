#********************************************
# @file size_calculat.cpp
# @author wangwenkai <qq:1162431386>
# @brief 测点匹配及长度计算
# @copyright Copyright (c) 2020
#********************************************
LIB_PATH =-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lpthread -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d -lm -Wall

Object= main.o pretreat.o size_calculat.o deepData.o

LIB_OBJ=pretreat.o size_calculat.o deepData.o

LIB_NAME ?=opencv_grad_pro

BULID_PATH:= ./bulid

AR=ar

ARFLAGS=rcs

#STATIC_NAME ?= lib$(LIB_NAME).a
#STA:$(LIB_OBJ)
#	$(AR) $(ARFLAGS) $(STATIC_NAME) $(LIB_OBJ) 
#	@echo "STATIC_TARGET MAKE SUCCESSFUL!"

edit: $(Object)
	g++ -o edit $(Object) $(LIB_PATH) 

main.o: pretreat.h opencv_hpp.h size_calculat.h deepData.h
pertreat.o: pretreat.h
size_calculat.o:size_calculat.h
deepData.o: deepData.h 



.PHONY:clean
clean:
	rm -f edit.o $(Object) $(STATIC_TARGET) 
