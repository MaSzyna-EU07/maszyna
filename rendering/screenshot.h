class screenshot_manager
{
    static void screenshot_save_thread(char *img , int w, int h);

public:
	// writes an image of the window to a png file in the directory of the screenshots, on a thread of its own. Image: rgb rows,
	// tightly packed, the bottom one first (taken over and deleted by the thread)
	static void save(char *Image, int Width, int Height);
};
