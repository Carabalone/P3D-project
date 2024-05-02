#pragma once
#include "scene.h"

class ComparePrimitives {
public:
	int sort_dim;
	
	ComparePrimitives(int sortDim) {
		sort_dim = sortDim;
	}

	bool operator()(Object *a, Object *b) {
		float ca = a->getCentroid().getAxisValue(sort_dim);
		float cb = b->getCentroid().getAxisValue(sort_dim);
		return ca < cb;
	}
};
