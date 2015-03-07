#include "ublib.h"

int main(void)
{
	init();
	while(1){
		loop();
	}
return 0;
}

void init() {
    initLeds();
    initMisc();
    //initButton();
}

void loop(){
	playLED();
}


