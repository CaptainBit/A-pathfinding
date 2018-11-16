#pragma once
#include <iostream>
#include <opencv2\opencv.hpp>
#include "Nodes.h"
#include <vector>

enum TypeNode {
	START, OBSTACLE, END
};
class NodesGrid
{
public:
	NodesGrid(int tabindex[7][7], bool step, cv::Mat board);


	int tabWithPath[7][7];
private:



	std::vector <Nodes*> ReconstructPath(cv::Point n);

	bool NearObstacle(cv::Point v, cv::Point p);

	std::vector <Nodes*> Start(bool step);


	cv::Mat _img;

	std::vector <Nodes*> lstOuverte;

	std::vector <Nodes*> lstFermee;


	void ShowResult();
	void ShowFinalResult(std::vector <Nodes*> path);


	void GetNearNodes(cv::Point p);

	void CalculateCostAndPush(cv::Point p, cv::Point v);

	int CalculateManhattanDistance(cv::Point p);

	int GetIndexMinCost();

	Nodes* tab[7][7];


	cv::Mat board;

	cv::Point start;

	cv::Point end;
};

