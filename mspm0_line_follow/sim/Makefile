# PC 仿真：gcc 编译（产品纯逻辑代码 + mock 物理模型）
CC      := gcc
CFLAGS  := -std=c99 -O2 -Wall -Wextra -I..
PRODUCT := ../app/line_task.c \
           ../control/line_controller.c \
           ../control/line_estimator.c \
           ../services/line_processing.c \
           ../drivers/encoder_speed.c
SIM     := sim_main.c sim_platform.c sim_track.c

sim_line: $(SIM) $(PRODUCT)
	$(CC) $(CFLAGS) -o $@ $(SIM) $(PRODUCT) -lm

clean:
	rm -f sim_line sim_line.exe
