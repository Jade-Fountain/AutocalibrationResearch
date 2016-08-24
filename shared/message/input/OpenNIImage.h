/*
 * This file is part of the Autocalibration Codebase.
 *
 * The Autocalibration Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Autocalibration Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Autocalibration Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */

#ifndef MESSAGE_INPUT_OPENNI_IMAGE_H
#define MESSAGE_INPUT_OPENNI_IMAGE_H

#include <OpenNI.h>

namespace message{
namespace input{

	class OpenNIImage{
	public:
		OpenNIImage(int w, int h) : width(w), height(h), imageData(w*h) {}
		std::vector<openni::RGB888Pixel> imageData;
		int width;
		int height;
		const openni::RGB888Pixel* data() const {return (const openni::RGB888Pixel*)&imageData[0];}
		openni::RGB888Pixel* data() {return &imageData[0];}

	};
	
}
}

#endif