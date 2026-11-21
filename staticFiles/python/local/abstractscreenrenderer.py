from PIL import Image, ImageDraw, ImageFont
from math import ceil, floor, cos, sin, radians
import builtins
import os
import sys
import time

# Linux tells the case of the letters in a file name apart, Windows doesn't, and the data is made on Windows: a screen asks for
# "WS_on.png" where the disk has "ws_on.png", or names a folder in another case. The simulator opens such files of its own through
# FileOnDisk(); the files a screen opens itself (images, fonts, text) are found here the same way. The name given when it's on the
# disk, otherwise the one differing from it only in case, of the file or of its folders. On Windows nothing changes.
_case_sensitive = not sys.platform.startswith('win')
_cased_listings = {}  # folder -> (when listed, {name in lower case: name on the disk})

def _cased_name(folder, name):
	"""the name in the folder differing from the one given only in case, as on the disk; None when there's none"""
	listed = _cased_listings.get(folder)
	# a name missing gets the folder listed again, at most once a second, as FileCased() does
	if listed is None or (name.lower() not in listed[1] and time.monotonic() - listed[0] >= 1.0):
		names = {}
		try:
			for entry in os.listdir(folder or '.'):
				names.setdefault(entry.lower(), entry)
		except OSError:
			pass
		listed = (time.monotonic(), names)
		_cased_listings[folder] = listed
	return listed[1].get(name.lower())

def file_on_disk(path):
	"""the file to open: as it's named on the disk, the path given when there's none (and on Windows, which finds it itself)"""
	if not _case_sensitive or not isinstance(path, str) or os.path.exists(path):
		return path
	text = path.replace('\\', '/')
	parts = [part for part in text.split('/') if part]
	cased = '/' if text.startswith('/') else ''
	for index, part in enumerate(parts):
		last = index == len(parts) - 1
		found = part
		if part not in ('.', '..') and not os.path.exists(cased + part):
			found = _cased_name(cased, part)
			if found is None:
				if not last:
					return path
				# a file not there yet goes into its folder as named on the disk: one written for the first time. The folder
				# gets listed anew next time, the file being there by then
				_cased_listings.pop(cased, None)
				return cased + part
		cased += found if last else found + '/'
	return cased

def _open_files_on_disk():
	# Image.open() opens the file through builtins.open(); the fonts are opened by FreeType, from the name. What the wrappers
	# call is held here, not looked up in __main__, which the screens share and could name the same
	on_disk = file_on_disk
	open_given = builtins.open
	truetype_given = ImageFont.truetype

	def open_on_disk(file, *args, **kwargs):
		return open_given(on_disk(file), *args, **kwargs)

	def truetype_on_disk(font=None, *args, **kwargs):
		return truetype_given(on_disk(font), *args, **kwargs)

	open_on_disk.file_on_disk = True
	builtins.open = open_on_disk
	ImageFont.truetype = truetype_on_disk

if _case_sensitive and not getattr(builtins.open, 'file_on_disk', False):
	_open_files_on_disk()

# Pillow 10 removed ImageFont.getsize(), ImageDraw.textsize() and Image.ANTIALIAS, which the screen scripts use; for
# Python 3.14 only a newer Pillow is available, so they are put back, on top of getbbox() / textbbox() and LANCZOS
def _font_getsize(self, text, *args, **kwargs):
	box = self.getbbox(text, *args, **kwargs)
	return (box[2], box[3])

def _draw_textsize(self, text, font=None, *args, **kwargs):
	box = self.textbbox((0, 0), text, font, *args, **kwargs)
	return (box[2], box[3])

for _fontclass in (ImageFont.FreeTypeFont, ImageFont.ImageFont, getattr(ImageFont, 'TransposedFont', None)):
	if _fontclass is not None and not hasattr(_fontclass, 'getsize') and hasattr(_fontclass, 'getbbox'):
		_fontclass.getsize = _font_getsize
if not hasattr(ImageDraw.ImageDraw, 'textsize'):
	ImageDraw.ImageDraw.textsize = _draw_textsize
if not hasattr(Image, 'ANTIALIAS'):
	Image.ANTIALIAS = Image.LANCZOS

class abstractscreenrenderer():
	def __init__(self, lookup_path):
		self.width = 0
		self.height = 0
		self.format = "RGB"
		print(lookup_path)

	def openimage(self, name):
		exts = [ ".png", ".tga", ".dds" ]
		for e in exts:
			path = file_on_disk(name + e)
			if os.path.isfile(path):
				if e == ".dds":
					return Image.open(path).transpose(Image.FLIP_TOP_BOTTOM)
				return Image.open(path)
		raise FileNotFoundError('image not found: ' + name)
	
	def rotate_and_paste(self, background, foreground, angle, pivot, center_to_pivot, scale_x = 1, scale_y =1):
		rad =  radians(angle)
		# wyznaczamy srodek wokol ktorego zostanie obrocony nakladany obraz	
		rotated = foreground.rotate(angle, expand=1)
		# wyznaczamy pozycje dla nowego srodka 
		rotated_center = ((center_to_pivot[0]*cos(rad)-center_to_pivot[1]*sin(rad))*scale_x,(center_to_pivot[0]*sin(rad)+center_to_pivot[1]*cos(rad))*scale_y)
		# wyznaczamy pozycje srodka na tle (pamietamy o odwroconej osi Y)
		center_postion = (pivot[0] + int(rotated_center[0]), pivot[1] - int(rotated_center[1]))
		# wyznaczamy obszar ktory zajmuje obrocony obrazek 		
		box = (center_postion[0] - int(ceil(rotated.size[0]/2)), center_postion[1] - int(ceil(rotated.size[1]/2)), center_postion[0] + int(floor(rotated.size[0]/2)), center_postion[1] + int(floor(rotated.size[1]/2)))
		background.paste(rotated, box, rotated)
		
	def print_fixed_with(self, draw, text, start_point, character_count, font, color, correction=0):
		rozmiar_osemki = font.getsize("8")[0]
		start_point_tmp = start_point[0]
		for znak in range(len(text), character_count):
			start_point_tmp += rozmiar_osemki-correction
		for znak in range(0,len(text)):
			rozmiar_act = font.getsize(text[znak])[0]
			draw.text((start_point_tmp + rozmiar_osemki - rozmiar_act, start_point[1]), text[znak], font=font, fill=color)
			start_point_tmp += rozmiar_osemki-correction
		
	def print_center(self, draw, text, X, Y, font, color):
		w = draw.textsize(text, font)
		draw.text(((X-w[0]/2),(Y-w[1]/2)), text, font=font, fill=color)

	def print_left(self, draw, text, X, Y, font, color):
		w = draw.textsize(text, font)
		draw.text((X,(Y-w[1]/2)), text, font=font, fill=color)

	def print_right(self, draw, text, X, Y, font, color):
		w = draw.textsize(text, font)
		draw.text(((X-w[0]),(Y-w[1]/2)), text, font=font, fill=color)
		
	def get_width(self):
		return self.__dict__['width']
	
	def get_height(self):
		return self.__dict__['height']

	def get_format(self):
		return self.format if self.format else "RGB"
	
	def render(self, state):
		image = self._render(state)
		self.__dict__['width'] = image.size[0]
		self.__dict__['height'] = image.size[1]
		image = image.transpose(Image.FLIP_TOP_BOTTOM)
		format = self.get_format()
		if image.mode != format:
			image = image.convert(format)
		return image.tobytes("raw", format)

	def getCommands(self):
		commands = self._getCommands()
		return commands

	def _getCommands(self):
		return []

	def manul_set_format(self, format_str):
		self.format = format_str
		return

	def _render(self, state):
		return Image.new("RGB", (1,1), color=(255,105,180))
	

