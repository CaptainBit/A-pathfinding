#pragma once
#include <iostream>
#include <opencv2\opencv.hpp>


enum Status {
	NONE, OPENLIST, CLOSEDLIST
};
class Nodes
{

public:
	Nodes(bool obstacle, cv::Point coord);
	Nodes();
	//Cost
	int _fCost;
	int _gCost;
	int _hCost;

	//Parent coord
	cv::Point _coordNodeParent;

	//Coord
	cv::Point _coord;


	//In the open list
	Status _status;

	//If node is an obstacle
	bool _obstacle;

};

