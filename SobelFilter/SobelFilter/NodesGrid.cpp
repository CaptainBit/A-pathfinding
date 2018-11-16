#include "NodesGrid.h"



NodesGrid::NodesGrid(int tabindex[7][7], bool step, cv::Mat boardMat)
{

	board = boardMat;

	

	for (int rows = 0; rows < 7; rows++) 
	{
		for (int cols = 0; cols < 7; cols++) 
		{
			
			if (tabindex[rows][cols] == OBSTACLE) {
				tab[rows][cols] = new Nodes(true, cv::Point(cols, rows));
			}
			else {
				tab[rows][cols] = new Nodes(false, cv::Point(cols, rows));
				if (tabindex[rows][cols] == START)
					start = cv::Point(cols, rows);
				else
					if (tabindex[rows][cols] == END)
						end = cv::Point(cols, rows);
			}
		}
	}
	std::vector <Nodes*> path = Start(step);
	ShowFinalResult(path);
}


void NodesGrid::GetNearNodes(cv::Point p)
{
	//Add Nodes to open list

	//Up
	CalculateCostAndPush(p, cv::Point(p.x - 1, p.y + 1));
	CalculateCostAndPush(p, cv::Point(p.x, p.y + 1));
	CalculateCostAndPush(p, cv::Point(p.x + 1, p.y + 1));

	//current row
	CalculateCostAndPush(p, cv::Point(p.x - 1, p.y));
	CalculateCostAndPush(p, cv::Point(p.x + 1, p.y));

	//Down
	CalculateCostAndPush(p, cv::Point(p.x - 1, p.y -1));
	CalculateCostAndPush(p, cv::Point(p.x, p.y - 1));
	CalculateCostAndPush(p, cv::Point(p.x + 1, p.y -1 ));
}

std::vector <Nodes*>  NodesGrid::Start(bool step)
{
	std::vector <Nodes*> path;
	//Add the start Point with cost of 0
	tab[start.y][start.x]->_fCost = 0;
	tab[start.y][start.x]->_gCost = 0;
	tab[start.y][start.x]->_hCost = 0;

	lstOuverte.push_back(tab[start.y][start.x]);

	while (lstOuverte.size() > 0) 
	{
		
		int index = GetIndexMinCost();
		Nodes* current = lstOuverte[index];

		tab[current->_coord.y][current->_coord.x]->_status = CLOSEDLIST;

		//Erase current in open list
		lstOuverte.erase(lstOuverte.begin() + index);
		
		//add the current in close 
		lstFermee.push_back(current);

		if (step == true) {
			ShowResult();
		}

		if (current->_coord == end) {
			//Chemin trouver
			path = ReconstructPath(current->_coord);
			return path;
		}
		
		GetNearNodes(current->_coord);


	}
	return path;
}

int NodesGrid::CalculateManhattanDistance(cv::Point p)
{
	int cost = 0;

	//Cost for x axis
	if (p.x > end.x)
		cost = cost + 10 * (p.x - end.x);
	else
		cost = cost + 10 * (end.x - p.x);

	//Cost for y axis
	if (p.y > end.y)
		cost = cost + 10 * (p.y - end.y);
	else
		cost = cost + 10 * (end.y - p.y);
	return cost;
}

void NodesGrid::CalculateCostAndPush(cv::Point p, cv::Point v)
{

	//if obstacle => dont put in open list and if coord not out of the array
	if (v.x >= 0 && v.x <= 6 && v.y >= 0 && v.y <= 6 && NearObstacle(v, p) == false) {
		if ((tab[v.y][v.x]->_status == NONE) && (tab[v.y][v.x]->_obstacle == false))
		{

			if (p.x != v.x && p.y != v.y)
			{
				tab[v.y][v.x]->_gCost = tab[p.y][p.x]->_gCost + 14;
			}
			else {
				tab[v.y][v.x]->_gCost = tab[p.y][p.x]->_gCost + 10;
			}
			tab[v.y][v.x]->_hCost = CalculateManhattanDistance(v);
			tab[v.y][v.x]->_fCost = tab[v.y][v.x]->_gCost + tab[v.y][v.x]->_hCost;
			tab[v.y][v.x]->_coordNodeParent = p;
			tab[v.y][v.x]->_status = OPENLIST;
			lstOuverte.push_back(tab[v.y][v.x]);

		}
		else //if the point v is already in the openlist => recalculate the g point
			if (tab[v.y][v.x]->_status == OPENLIST)
			{
				int gCost;

				if (p.x != v.x && p.y != v.y)
				{
					gCost = tab[p.y][p.x]->_gCost + 14;
				}
				else {
					gCost = tab[p.y][p.x]->_gCost + 10;
				}

				//change parent node if new gcost is lower than current gcost
				if (tab[v.y][v.x]->_gCost > gCost) {
					tab[v.y][v.x]->_gCost = gCost;
					tab[v.y][v.x]->_coordNodeParent = p;
				}
			}
		return;
	}
	return;
	
}

int NodesGrid::GetIndexMinCost() 
{
	int minCost = lstOuverte[0]->_fCost;
	int index = 0;
	for (int i = 0; i < lstOuverte.size(); i++)
	{
		if (minCost > lstOuverte[i]->_fCost) {
			minCost = lstOuverte[i]->_fCost;
			index = i;
		}
	}
	return index;
}

bool NodesGrid::NearObstacle(cv::Point v, cv::Point p) {

	if (v.x > p.x) {

		if (v.y > p.y) {
	
			if (v.y < 6) {
				if (tab[v.y + 1][v.x]->_obstacle == true) {
					return true;
				}
			}
			if (v.x < 6) {
				if (tab[v.y][v.x + 1]->_obstacle == true) {
					return true;
				}
			}
		}
		else
		{
			if (v.x < 6) {
				if (tab[p.y][p.x + 1]->_obstacle == true) {
					return true;
				}
			}
			if (v.y > 1) {
				if (tab[v.y - 1][v.x]->_obstacle == true) {
					return true;
				}
			}
		}
	}
	else {
		if (v.y > p.y) {

			if (tab[v.y + 1][v.x]->_obstacle == true || tab[v.y][v.x - 1]->_obstacle == true) {
				return true;
			}

		}
		else
		{
			if (tab[p.y][p.x - 1]->_obstacle == true || tab[v.y - 1][v.x]->_obstacle == true) {
				return true;
			}
		}
	}
	return false;
}


std::vector <Nodes*> NodesGrid::ReconstructPath(cv::Point n)
{

	Nodes* current = tab[n.y][n.x];
	cv::Point p = n;
	std::vector <Nodes*> Path;


	while (p != start) 
	{
		p = current->_coordNodeParent;
		Path.push_back(current);

		current = tab[p.y][p.x];
	}
	Path.push_back(tab[p.y][p.x]);
	return Path;
}



void NodesGrid::ShowResult() 
{
	float nbPRows = board.rows / 7;
	float nbPCols = board.cols / 7;


	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 7; j++)
		{
			Status status = tab[i][j]->_status;
			std::string parentCoord = "Parent : " + std::to_string(tab[i][j]->_coordNodeParent.x) + " , " + std::to_string(tab[i][j]->_coordNodeParent.y);
			std::string f = "F : " + std::to_string(tab[i][j]->_fCost);
			std::string g = "G : " + std::to_string(tab[i][j]->_gCost);
			std::string h = "H : " + std::to_string(tab[i][j]->_hCost);
			if (status == Status::OPENLIST) {
				cv::circle(board, cv::Point((nbPCols * j) + nbPCols / 2.0f, (nbPRows * i) + nbPRows / 2.0f), 12.0, cv::Scalar(0, 0, 255), -1, 8);

				
				//Affichage Texts
				putText(board, parentCoord, cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 15), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
				putText(board, f, cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 30), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
				putText(board,g, cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 45), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
				putText(board, h , cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 60), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
			}
			else {
				if (status == Status::CLOSEDLIST) {
					cv::circle(board, cv::Point((nbPCols * j) + nbPCols / 2.0f, (nbPRows * i) + nbPRows / 2.0f), 12.0, cv::Scalar(255, 0, 0), -1, 8);

					//Affichage Texts
					putText(board, parentCoord, cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 15), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
					putText(board, f, cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 30), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
					putText(board, g, cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 45), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
					putText(board, h, cv::Point((nbPCols * (j - 1)) + nbPCols + 5, (nbPRows * (i)) + 60), CV_FONT_NORMAL, 0.5, cv::Scalar(0, 0, 0), 1);
				}
			}
			
		}
		
	}
	cv::imshow("step", board);
	cv::waitKey(0);
	
}

void NodesGrid::ShowFinalResult(std::vector <Nodes*> path)
{
	float nbPRows = board.rows / 7;
	float nbPCols = board.cols / 7;


	for (int i = 0; i < path.size(); i++)
	{
		cv::circle(board, cv::Point((nbPCols * path[i]->_coord.x) + nbPCols / 2.0f, (nbPRows *  path[i]->_coord.y) + nbPRows / 2.0f), 12.0, cv::Scalar(0, 255, 0), -1, 8);

	}
	cv::imshow("step", board);
	cv::waitKey(0);
}