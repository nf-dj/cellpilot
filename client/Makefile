raspilot: raspilot.c
	gcc -o $@ $< -llua5.1 -lm -ldl -lavcodec -lswscale -lSDL2 -lSDL2_ttf -lSDL2_gfx

clean:
	rm -f raspilot
