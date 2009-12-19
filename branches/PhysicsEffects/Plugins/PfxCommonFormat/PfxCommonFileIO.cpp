/*
   Copyright (C) 2009 Sony Computer Entertainment Inc.
   All rights reserved.

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

*/

#include "Plugins/PfxCommonFormat/PfxCommonFileIO.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

bool PfxCommonFileReader::open(const char *filename)
{
	mFp = fopen(filename,"r");
	if(!mFp) return false;
	return true;
}

bool PfxCommonFileReader::getNextTag()
{
	if(isEof()) return false;
	getLine(mLineBuff,LINEBUFF_MAX);
	readTag(mLineBuff,mTag);
	return true;
}

bool PfxCommonFileReader::isSameTag(char *tag)
{
	return isSame(mTag,tag);
}

void PfxCommonFileReader::getLine(char *line,int n)
{
	if(mFp) fgets(line,n,mFp);
}

bool PfxCommonFileReader::isSame(char *s1,char *s2)
{
	return strcmp(s1,s2)==0;
}

bool PfxCommonFileReader::isToken(char c)
{
	return (c==' ' || c=='\t' || c=='\n' || c=='\r' || c==',');
}

void PfxCommonFileReader::readTag(char *line,char *tag)
{
	char *cur = line;
	char *s,*e;
	s = e = NULL;

	tag[0] = '\0';
	while(*cur!='\0') {
		if(!isToken(*cur) && !s && !e) {
			s = cur;
		}
		if(isToken(*cur) && s && !e){
			e = cur;
			assert(e-s<=TAG_STR_MAX);
			strncpy(tag,s,e-s);
			tag[e-s] = '\0';
			return;
		}
		cur++;
	}
}

void PfxCommonFileReader::readString(char *str,int n)
{
	char *cur = mLineBuff;
	char *s,*e;
	s = e = NULL;

	str[0] = '\0';
	while(*cur!='\0') {
		if(*cur=='\"' && !s && !e) {
			s = ++cur;
		}
		if(*cur=='\"' && s && !e){
			e = cur;
			int sz = MIN(e-s,n-1);
			strncpy(str,s,sz);
			str[sz] = '\0';
			return;
		}
		cur++;
	}
}

void PfxCommonFileReader::readUInt32(PfxUInt32 &value)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %u",dummy,&value);
}

void PfxCommonFileReader::readBool(bool &value)
{
	PfxUInt32 tmp;
	readUInt32(tmp);
	value = (bool)tmp;
}

void PfxCommonFileReader::readInt32(PfxInt32 &value)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %d",dummy,&value);
}

void PfxCommonFileReader::readFloat(PfxFloat &value)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %f",dummy,&value);
}

void PfxCommonFileReader::readFloat3(PfxFloat3 &value)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %f %f %f\n",dummy,&value[0],&value[1],&value[2]);
}

void PfxCommonFileReader::readFloat4(PfxFloat4 &value)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %f %f %f %f\n",dummy,&value[0],&value[1],&value[2],&value[3]);
}

void PfxCommonFileReader::readMatrix3(PfxMatrix3 &value)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %f %f %f %f %f %f %f %f %f\n",dummy,
		&value[0][0],&value[0][1],&value[0][2],
		&value[1][0],&value[1][1],&value[1][2],
		&value[2][0],&value[2][1],&value[2][2]);
}

void PfxCommonFileReader::readIndex3(PfxUInt32 &value0,PfxUInt32 &value1,PfxUInt32 &value2)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %d %d %d\n",dummy,&value0,&value1,&value2);
}

void PfxCommonFileReader::readFloat3(PfxFloat &value0,PfxFloat &value1,PfxFloat &value2)
{
	char dummy[TAG_STR_MAX];
	sscanf(mLineBuff,"%s %f %f %f\n",dummy,&value0,&value1,&value2);
}


bool PfxCommonFileWriter::open(const char *filename)
{
	mFp = fopen(filename,"w");
	if(!mFp) return false;
	mIndentCnt = 0;
	mIndents[0] = '\0';
	return true;
}


void PfxCommonFileWriter::indentUp()
{
	assert(mIndentCnt < INDENT_MAX);
	mIndents[mIndentCnt] = '\t';
	mIndents[++mIndentCnt] = '\0';
}

void PfxCommonFileWriter::indentDown()
{
	assert(mIndentCnt > 0);
	mIndents[--mIndentCnt] = '\0';
}

void PfxCommonFileWriter::writeOpenTag(const char *name)
{
	fprintf(mFp,"%s%s\n",mIndents,name);
	fprintf(mFp,"%s{\n",mIndents);
	indentUp();
}

void PfxCommonFileWriter::writeCloseTag()
{
	indentDown();
	fprintf(mFp,"%s}\n",mIndents);
}

void PfxCommonFileWriter::writeString(const char *name,const char *str)
{
	fprintf(mFp,"%s%s \"%s\"\n",mIndents,name,str);
}

void PfxCommonFileWriter::writeBool(const char *name,bool value)
{
	writeUInt32(name,(PfxUInt32)value);
}

void PfxCommonFileWriter::writeUInt32(const char *name,PfxUInt32 value)
{
	fprintf(mFp,"%s%s %u\n",mIndents,name,value);
}

void PfxCommonFileWriter::writeInt32(const char *name,PfxInt32 value)
{
	fprintf(mFp,"%s%s %d\n",mIndents,name,value);
}

void PfxCommonFileWriter::writeFloat(const char *name,PfxFloat value)
{
	fprintf(mFp,"%s%s %.5f\n",mIndents,name,value);
}

void PfxCommonFileWriter::writeFloat3(const char *name,const PfxFloat3 &value)
{
	fprintf(mFp,"%s%s %.5f %.5f %.5f\n",mIndents,name,
		value[0],value[1],value[2]);
}

void PfxCommonFileWriter::writeMatrix3(const char *name,const PfxMatrix3 &value)
{
	fprintf(mFp,"%s%s %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n",mIndents,name,
		value[0][0],value[0][1],value[0][2],  // col0
		value[1][0],value[1][1],value[1][2],  // col1
		value[2][0],value[2][1],value[2][2]); // col2
}

void PfxCommonFileWriter::writeFloat4(const char *name,const PfxFloat4 &value)
{
	fprintf(mFp,"%s%s %.5f %.5f %.5f %.5f\n",mIndents,name,
		value[0],value[1],value[2],value[3]);
}

void PfxCommonFileWriter::writeIndex3(const char *name,PfxUInt32 value0,PfxUInt32 value1,PfxUInt32 value2)
{
	fprintf(mFp,"%s%s %u %u %u\n",mIndents,name,value0,value1,value2);
}

void PfxCommonFileWriter::writeFloat3(const char *name,PfxFloat value0,PfxFloat value1,PfxFloat value2)
{
	fprintf(mFp,"%s%s %.5f %.5f %.5f\n",mIndents,name,value0,value1,value2);
}
