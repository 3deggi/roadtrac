/* Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
/* File for "Drawing Text" lesson of the OpenGL tutorial on
 * www.videotutorialsrock.com
 */

//! by songkran

#ifndef TEXT_3D_H_INCLUDED
#define TEXT_3D_H_INCLUDED

#include <fstream>
#include <string>


//Indicates that an exception occurred when setting up 3D text
class T3DLoadException {
	private:
		std::string message0;
	public:
		T3DLoadException(std::string message1);
		std::string message() const;
};


//Just like auto_ptr, but for arrays
template<class T>
class auto_array {
	private:
		T* array;
		mutable bool isReleased;
	public:
		explicit auto_array(T* array_ = NULL) :
			array(array_), isReleased(false) {
		}
		
		auto_array(const auto_array<T> &aarray) {
			array = aarray.array;
			isReleased = aarray.isReleased;
			aarray.isReleased = true;
		}
		
		~auto_array() {
			if (!isReleased && array != NULL) {
				delete[] array;
			}
		}
		
		T* get() const {
			return array;
		}
		
		T &operator*() const {
			return *array;
		}
		
		void operator=(const auto_array<T> &aarray) {
			if (!isReleased && array != NULL) {
				delete[] array;
			}
			array = aarray.array;
			isReleased = aarray.isReleased;
			aarray.isReleased = true;
		}
		
		T* operator->() const {
			return array;
		}
		
		T* release() {
			isReleased = true;
			return array;
		}
		
		void reset(T* array_ = NULL) {
			if (!isReleased && array != NULL) {
				delete[] array;
			}
			array = array_;
		}
		
		T* operator+(int i) {
			return array + i;
		}
		
		T &operator[](int i) {
			return array[i];
		}
};

enum Opcodes {OP_NORMAL = 65532,
			  OP_TRIANGLE_STRIP,
			  OP_TRIANGLES,
			  OP_END_PART};

const float PI_TIMES_2_OVER_65536 = 2 * 3.1415926535f / 65536.0f;
//#define PI_TIMES_2_OVER_65536  (2 * 3.1415926535f / 65536.0f)


class T3DFont 
{
private:
	int toInt(const char* bytes);
	float toFloat(const char* buffer);
	unsigned short toUShort(const char* buffer);

	float spaceWidth;
	float widths[94];
	GLuint displayListId2D;
	GLuint displayListId3D;
public:
	//Loads the specified font file into a new T3DFont object
	T3DFont (std::ifstream &input);
	void draw2D(char c) {
		if (c >= 33 && c <= 126) {
			glCallList(displayListId2D + c - '!');
		}
	}
	
	void draw3D(char c) {
		if (c >= 33 && c <= 126) {
			glCallList(displayListId3D + c - '!');
		}
	}
	
	float width(char c) {
		if (c >= 33 && c <= 126) {
			return widths[c - 33];
		}
		else {
			return spaceWidth;
		}
	}

	

};

class T3DText
{
public:
	T3DText();
	virtual~T3DText();
	void t3dDraw2D(std::string str,
				   int hAlign, int vAlign,
				   float lineHeight = 1.5f);
	/* Draws the specified string, using OpenGL, using polygons as a right prism,
	 * where the parallel faces are letters parallel to the x-y plane, with the top
	 * of the letters having the greatest y coordinate.
	 * 
	 * The string is drawn left-aligned if hAlign is negative, right-aligned if it
	 * is positive, and centered horizontally if it is 0.  The string is drawn top-
	 * aligned if vAlign is negative, bottom-aligned if it is positive, and centered
	 * vertically if it is 0.
	 * 
	 * The string may have newline characters, in which case the string will be
	 * drawn on multiple lines as one would expect.  The lines are drawn lineHeight
	 * times the height of the font apart.  The height of the font is the "normal"
	 * height of capital letters, rather than the distance from the top of "normal"
	 * capital letters to the bottom of lowercase letters like "p".
	 * 
	 * The depth of the characters is depth times the height of the font.  The
	 * characters are centered at z = 0.
	 * 
	 * All unprintable ASCII characters (other than '\n') are drawn as spaces.
	 */
	void t3dDraw3D(std::string str,
				   int hAlign, int vAlign,
				   float depth,
				   float lineHeight = 1.5f);
	/* Returns the draw width of the specified string, as a multiple of the height
	 * of the font.  The height of the font is the "normal" height of capital
	 * letters, rather than the distance from the top of "normal" capital letters to
	 * the bottom of lowercase letters like "p".  The width is the same as the width
	 * of the longest line.
	 */
	float t3dDrawWidth(std::string str);
	/* Returns the draw height of the specified string, as a multiple of the height
	 * of the font.  The height of the font is the "normal" height of capital
	 * letters, rather than the distance from the top of "normal" capital letters to
	 * the bottom of lowercase letters like "p".  The draw is lineHeight times one
	 * fewer than the number of lines in the string, plus 1.
	 */
	float t3dDrawHeight(std::string str, float lineHeight = 1.5f);
	void t3dInit(const char* name);

private:
	enum {DRAW2D = 0, DRAW3D = 1};

	//Initializes 3D text.  Must be called before other functions in this header.
	//Frees memory allocated for 3D text.  No other functions in this header may be
	//called after this one.
	void t3dCleanup();
	/* Draws the specified string, using OpenGL, as a set of polygons in the x-y
	 * plane, with the top of the letters having the greatest y coordinate.  The
	 * normals point in the positive z direction.  (If you need the normals to point
	 * in the positive z direction on one side of the characters and the negative z
	 * direction on the other, call t3dDraw3D with a very small depth.)
	 * 
	 * The string is drawn left-aligned if hAlign is negative, right-aligned if it
	 * is positive, and centered horizontally if it is 0.  The string is drawn top-
	 * aligned if vAlign is negative, bottom-aligned if it is positive, and centered
	 * vertically if it is 0.
	 * 
	 * The string may have newline characters, in which case the string will be
	 * drawn on multiple lines as one would expect.  The lines are drawn lineHeight
	 * times the height of the font apart.  The height of the font is the "normal"
	 * height of capital letters, rather than the distance from the top of "normal"
	 * capital letters to the bottom of lowercase letters like "p".
	 * 
	 * All unprintable ASCII characters (other than '\n') are drawn as spaces.
	 */


	void draw2D(char c);
	void draw3D(char c);
	void drawLine(const char* str, int hAlign, int drawMode);
	void draw(const char* str,
		  int hAlign, int vAlign,
		  float lineHeight,
		  int drawMode);



	T3DFont* font; //The font used to draw 2D and 3D characters
};




#endif
