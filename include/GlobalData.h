#pragma once

#include "Audio.h"

class GlobalData
{
	private:
		//! Singleton implementation
		GlobalData() {}

	public:
		static GlobalData& getSingleton() { static GlobalData data; return data; }

		Audio mAudio;
};

