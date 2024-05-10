//#include "rayAccelerator.h"
//#include "macros.h"
//
//bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
//	// https://github.com/madmann91/bvh/blob/master/include/bvh/single_ray_traverser.hpp
//
//	float tmp;
//	float tmin = FLT_MAX;  //contains the closest primitive intersection
//	bool hit = false;
//
//	BVHNode* currentNode = nodes[0];
//
//	// If our ray doesn't intercept the first node, it won't intercept any other
//	if (!currentNode->getAABB().intercepts(ray, tmp)) {
//		return false;
//	}
//
//	BVHNode* l_child;
//	BVHNode* r_child;
//
//	while (true) {
//		// If the root is a leaf, intersect it
//		if (currentNode->isLeaf()) {
//			Object* obj;
//			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
//				obj = objects[i];
//				if (obj->intercepts(ray, tmp) && tmp < tmin) {
//					tmin = tmp;
//					*hit_obj = obj;
//
//				}
//			}
//
//			if (hit_obj != NULL) {
//				hit = true;
//			}
//		}
//		else {
//			l_child = nodes[currentNode->getIndex()];
//			r_child = nodes[currentNode->getIndex() + 1];
//
//			float l_dist, r_dist;
//
//			bool l_hit = l_child->getAABB().intercepts(ray, l_dist);
//			bool r_hit = r_child->getAABB().intercepts(ray, r_dist);
//
//			if (l_child->getAABB().isInside(ray.origin)) l_dist = 0;
//			if (r_child->getAABB().isInside(ray.origin)) r_dist = 0;
//
//			// If intersection happened, but at a distance larger than the current minimum, we don't care about it
//			if (l_hit && l_dist > tmin)
//				l_hit = false;
//
//			if (r_hit && r_dist > tmin)
//				r_hit = false;
//
//			if (l_hit && r_hit) { // If both hit, pick the closest, store the other
//				if (l_dist < r_dist) { // Distance to left node is smaller, so move to that one and store the other
//					currentNode = l_child;
//					hit_stack.push(StackItem(r_child, r_dist));
//				}
//				else { // Distance to right node is smaller, so move to that one and store the other
//					currentNode = r_child;
//					hit_stack.push(StackItem(l_child, l_dist));
//				}
//				continue;
//			}
//			else if (l_hit) { // If only left hits, pick it
//				currentNode = l_child;
//				continue;
//			}
//			else if (r_hit) { // if only right hits, pick it
//				currentNode = r_child;
//				continue;
//			}
//		}
//
//		// If no new node or already explored leaf, get from the stack (or break if empty)
//		bool newNode = false;
//
//		while (!hit_stack.empty()) {
//			StackItem popped = hit_stack.top();
//			hit_stack.pop();
//
//			if (popped.t < tmin) { // If Intersection to box is larger than our closest intersection to an object, so continue
//				currentNode = popped.ptr;
//				newNode = true;
//				break;
//			}
//		}
//
//		if (!newNode) // No new node was found (stack is empty or no node in stack is closer than our closest intersection
//			break;
//	}
//
//	if (hit) { // If by the end we found a hit, compute the hit point
//		hit_point = ray.origin + ray.direction * tmin;
//		return true;
//	}
//
//	return false;
//}
