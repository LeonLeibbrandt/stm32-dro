/**
 * original author:  Tilen Majerle<tilen@majerle.eu>
 * modification for STM32f10x: Alexander Lutsai<s.lyra@ya.ru>
   ----------------------------------------------------------------------
   	Copyright (C) Alexander Lutsai, 2016
    Copyright (C) Tilen Majerle, 2015
    
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    any later version.
     
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    
    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
   ----------------------------------------------------------------------
 */
#ifndef FONTS_H
#define FONTS_H

#include <inttypes.h>

typedef struct {
	uint8_t FontWidth;    /*!< Font width in pixels */
	uint8_t FontHeight;   /*!< Font height in pixels */
	uint8_t CharBytes;    /*!< Count of bytes for one character */
	const uint8_t *data; /*!< Pointer to data font data array */
} FontDef_t;

typedef struct {
	uint16_t Length;      /*!< String length in units of pixels */
	uint16_t Height;      /*!< String height in units of pixels */
} FONTS_SIZE_t;


extern FontDef_t Font_12x24;

char* FONTS_GetStringSize(char* str, FONTS_SIZE_t* SizeStruct, FontDef_t* Font);

#endif
