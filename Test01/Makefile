#********************************************
# @file size_calculat.cpp
# @author wangwenkai <qq:1162431386>
# @brief 测点匹配及长度计算
# @copyright Copyright (c) 2020
#********************************************
LIB_PATH =-lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio -lpthread -lopencv_imgcodecs -lopencv_calib3d -lopencv_features2d -lm -Wall

Object= main.o 

#STATIC_NAME ?= lib$(LIB_NAME).a
#STA:$(LIB_OBJ)
#	$(AR) $(ARFLAGS) $(STATIC_NAME) $(LIB_OBJ) 
#	@echo "STATIC_TARGET MAKE SUCCESSFUL!"

edit: $(Object)
	g++ -o edit $(Object) $(LIB_PATH) 

main.o:


.PHONY:clean
clean:
	rm -f edit.o $(Object) $(STATIC_TARGET) 
