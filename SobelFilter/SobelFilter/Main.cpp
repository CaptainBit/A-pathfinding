#include"NodesGrid.h"
#include "stdafx.h"
#include <Windows.h>

int main()
{
	int tabindex[7][7]{
		{3,3,3,2,3,3,3},
		{1,1,1,1,1,3,3},
		{3,3,3,3,3,3,1},
		{1,3,3,3,3,3,3},
		{3,3,1,1,3,1,3},
		{3,3,3,3,3,3,3},
		{1,1,3,0,3,3,1}
	};
	NodesGrid nd(tabindex);
	for (int i = 0; i < 7; i++) 
	{
		for (int j = 0; j < 7; j++)
		{
			if (nd.tabWithPath[i][j] == 5 || nd.tabWithPath[i][j] == 0) {
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN);
			}
			else if(nd.tabWithPath[i][j]  == 1){
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_RED);
			}
			else if(nd.tabWithPath[i][j] == 2){
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_BLUE);
			}
			else {
				SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_INTENSITY);
			}
			std::cout << nd.tabWithPath[i][j] << ' ';
		}
		std::cout << std::endl;
	}
	cv::waitKey(0);
	return 0;
}