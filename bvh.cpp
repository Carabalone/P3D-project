#include "rayAccelerator.h"
#include "macros.h"
#include "comparePrimitives.h"

using namespace std;

enum Axis {
	X = 0,
	Y = 1,
	Z = 2
};

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
			//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

bool getBiggestPair(std::pair<Axis, float> p1, std::pair<Axis, float> p2) {
	return p1.second < p2.second;
}

std::pair<Axis, float> getBiggestAxis(AABB bbox) {


	std::pair<Axis, float> xDist = std::pair<Axis, float>(Axis::X, bbox.max.x - bbox.min.x);
	std::pair<Axis, float> yDist = std::pair<Axis, float>(Axis::Y, bbox.max.y - bbox.min.y);
	std::pair<Axis, float> zDist = std::pair<Axis, float>(Axis::Z, bbox.max.z - bbox.min.z);

	return std::max(std::max(xDist, yDist, getBiggestPair), zDist, getBiggestPair);
}

void BVH::build_recursive(int left_index, int right_index, BVHNode *node) {
	   //PUT YOUR CODE HERE

	if ((right_index - left_index) <= Threshold) {
		node->makeLeaf(left_index, right_index - left_index);
	}
	else {
		//std::pair<Axis, float> biggestAxisDist = getBiggestAxis(node->getAABB());
		//Axis axis = biggestAxisDist.first;
		//float midPoint = node->getAABB().min.getAxisValue(axis) + std::fabs(biggestAxisDist.second / 2.0f);

		Vector dist = node->getAABB().max - node->getAABB().min;
		Axis axis;

		if (dist.x >= dist.y && dist.x >= dist.z)
			axis = Axis::X;
		else if (dist.y >= dist.x && dist.y >= dist.z)
			axis = Axis::Y;
		else
			axis = Axis::Z;

		float midPoint = (node->getAABB().max.getAxisValue(axis) + node->getAABB().min.getAxisValue(axis)) / 2.0f;

		ComparePrimitives cmp = ComparePrimitives(axis);
		if (left_index > objects.size() || right_index > objects.size()) {
			std::cout << "ACCESS VIOLATION\n";
		}
		std::sort(objects.begin() + left_index, objects.begin() + right_index, cmp);

		// check if either side is empty
		if (objects[left_index]->getCentroid().getAxisValue(axis) > midPoint ||
			objects[right_index - 1]->getCentroid().getAxisValue(axis) < midPoint) {

			float sum = 0.0f;
			for (int i = left_index; i < right_index; i++) {
				sum += objects[i]->getCentroid().getAxisValue(axis);
			}

			midPoint = sum / (right_index - left_index);
		}

		//TODO substitute for binary search
		int i = 0, split_index = -1;
		for (int i = left_index; i < right_index; i++) {
			auto object = objects[i];

			auto val = object->getCentroid().getAxisValue(axis);
			if (object->getCentroid().getAxisValue(axis) > midPoint) {
				split_index = i;
				break;
			}
			i++;
		}

		// if split index has less then a leaf on one side, then we need to recompute it
		// Example: [0, 1, 2, 3, 4, 5, 6, 7, 8, 9], left_index = 0, right_index = 6
		// if split_index = 1, it is less than 2. We need to recompute it.
		// if split_index = 6, we need to recompute it.
		if (split_index == -1 || split_index < left_index + 2 || split_index > right_index - 1) {
			split_index = left_index + (right_index - left_index) / 2;
		}

		std::cout << "Axis: " << axis << "\n";
		std::cout << "Midpoint: " << midPoint << "\n";
		std::cout << "Split index: " << split_index << "\n";

		Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		AABB leftAABB = AABB(min, max);
		AABB rightAABB = AABB(min, max);

		for (int i = left_index; i < right_index; i++) {
			if (i < split_index) {
				leftAABB.extend(objects[i]->GetBoundingBox());
			}
			else {
				rightAABB.extend(objects[i]->GetBoundingBox());
			}
		}

		BVHNode* leftNode = new BVHNode();
		BVHNode* rightNode = new BVHNode();

		leftNode->setAABB(leftAABB);
		rightNode->setAABB(rightAABB);

		node->makeNode(nodes.size());

		nodes.push_back(leftNode);
		nodes.push_back(rightNode);

		build_recursive(left_index, split_index, leftNode);
		build_recursive(split_index, right_index, rightNode);
	}


	//right_index, left_index and split_index refer to the indices in the objects vector
   // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	// node.index can have a index of objects vector or a index of nodes vector
		
	
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {

	struct NodeInfo {
		BVHNode* node;
		float t;
	};

	float tmp;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;
	Ray localRay = ray;
	float t;

	BVHNode* currentNode = nodes[0];

	std::stack<NodeInfo> stack;

	if (!currentNode->getAABB().intercepts(localRay, t)) {
		return false;
	}

	while (true) {
		if (!currentNode->isLeaf()) {
			NodeInfo leftNodeInfo = { nodes[currentNode->getIndex()], 0.0f };
			NodeInfo rightNodeInfo = { nodes[currentNode->getIndex() + 1], 0.0f };

			bool rightHit = rightNodeInfo.node->getAABB().intercepts(localRay, rightNodeInfo.t);
			if (leftNodeInfo.node->getAABB().intercepts(localRay, leftNodeInfo.t)) {
				if (rightHit) {
					// both hit, put the farthest on the stack
					if (leftNodeInfo.t > rightNodeInfo.t) {
						stack.push(leftNodeInfo);
						currentNode = rightNodeInfo.node;
					}
					else {
						stack.push(rightNodeInfo);
						currentNode = leftNodeInfo.node;
					}
				} 
				else { // only left hits
					currentNode = leftNodeInfo.node;
				}
			}
			else if (rightHit) { // only right hits
				currentNode = rightNodeInfo.node;
			}
		}
		else { // is leaf
			auto total = currentNode->getIndex() + currentNode->getNObjs();
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects[i]->intercepts(localRay, tmin)) {
					hit = true;
					hit_obj = &objects[i];
				}
			}
		}

		bool found = false;
		while (!stack.empty()) {
			NodeInfo nodeInfo = stack.top();
			stack.pop();
			
			if (nodeInfo.t < tmin) {
				found = true;
				currentNode = nodeInfo.node;
				break;
			}
		}

		if (!found) {
			break;
		}
	}

	if (hit) {
		hit_point = localRay.origin + localRay.direction * tmin;
		return true;
	}
	
	return(false);
}

bool BVH::Traverse(Ray& ray) {  //shadow ray with length
	float tmp;

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	struct NodeInfo {
		BVHNode* node;
		float t;
	};

	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;
	Ray localRay = ray;
	float t;

	BVHNode* currentNode = nodes[0];

	std::stack<NodeInfo> stack;

	if (!currentNode->getAABB().intercepts(localRay, t)) {
		return false;
	}

	while (true) {
		if (!currentNode->isLeaf()) {
			NodeInfo leftNodeInfo = { nodes[currentNode->getIndex()], 0.0f };
			NodeInfo rightNodeInfo = { nodes[currentNode->getIndex() + 1], 0.0f };

			bool rightHit = rightNodeInfo.node->getAABB().intercepts(localRay, rightNodeInfo.t);
			if (leftNodeInfo.node->getAABB().intercepts(localRay, leftNodeInfo.t)) {
				if (rightHit) {
					// both hit, put the right one on the stack
					stack.push(rightNodeInfo);
					continue;
				} 
				else { // only left hits
					currentNode = leftNodeInfo.node;
					continue;
				}
			}
			else if (rightHit) { // only right hits
				currentNode = rightNodeInfo.node;
				continue;
			}
		}
		else { // is leaf
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				if (objects[i]->intercepts(localRay, tmin)) {
					return true;
				}
			}
		}

		if (stack.empty()) {
			return false;
		}
		auto poppedValue = stack.top();
		stack.pop();
		currentNode = poppedValue.node;

	}

	return(false);
}		
