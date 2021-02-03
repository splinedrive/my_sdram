my_sdram is a simple implementation of a sdram controller. 
This implementation works on a blackicemx currently. It was not my
intention to provide this code. I learn from articles and other code
how to do things in FPGA world and later I write them from scratch again to
understand it. The goal is to have
a memory controller because I wrote a hdmi controller and I want to use
it later with a videobuffer. The next days I will optimize the statemachine to avoid autoprecharging. 
And then I could use it as a video framebuffer. 
This implementation based on knowledge and ideas from 
http://www.fpga4fun.com/SDRAM.html and git@github.com:mcleod-ideafix/simple_sdram_controller.git and from pdfs I have committed
under docs. Let's dive in into the memory world. :)

[![circuit shortcut](https://github.com/splinedrive/my_sdram/blob/main/my_sdram.gif)
