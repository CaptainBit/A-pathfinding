
#include "cuda_runtime.h" 
#include "device_launch_parameters.h"
#include "stdafx.h"

#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/types_c.h> 
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <sstream>
#include <windows.h>
#include <vector>
#include <string>

#include "AxisCommunication.h"

#include "NodesGrid.h"

using namespace cv;
using namespace std;

extern "C" void EraseBackground(Mat *ptMatA, Mat *ptMatR, uchar backgroundColor[], uchar foregroundColor[], int threshMin, int threshMax);
//extern "C" void ApplySobelFilter(Mat *ptMatA, Mat *ptMatR, double mult);

void FlushCameraBuffer(Axis cam)
{
	Mat matTempo;

	for (int i = 0; i < 300; i++)
	{
		cam.GetImage(matTempo);
	}
}
int main()
{

	Mat img;
	Mat img2;
	Mat imgdestfinal;


	//img = imread("Images/Vision2.jpg");
	//img2 = imread("Images/Vision1.jpg");

	Axis axis("10.128.3.4", "etudiant", "gty970");

	axis.AbsolutePan(136.67);
	Sleep(2000);
	axis.AbsoluteTilt(-70.70);

	Sleep(3000);
	axis.GetImage(img);
	/*imwrite("Images/Vision1.jpg", img);*/

	Sleep(2000);

	FlushCameraBuffer(axis);

	Sleep(5000);

	axis.AbsolutePan(-44.99);
	Sleep(2000);
	axis.AbsoluteTilt(-66.10);

	Sleep(3000);
	axis.GetImage(img2);
	/*imwrite("Images/Vision2.jpg", img2);*/

	axis.ReleaseCam();

	//reforme les images pour qu'il soients de même grosseur
	resize(img, img, Size(1024, 400), 0, 0, INTER_CUBIC);
	resize(img2, img2, Size(1024, 400), 0, 0, INTER_CUBIC);

	if (!img.data)
	{
		cout << "Aucune image de disponible" << std::endl;
		return -1;
	}

	//inverse l'img2
	flip(img2, img2, -1);

	////copy img et img2 dans imgdestfinal
	imgdestfinal.push_back(img);
	imgdestfinal.push_back(img2);

	//reforme l'image
	resize(imgdestfinal, imgdestfinal, Size(1024, 800), 0, 0, INTER_CUBIC);


	////Erase background
	Mat imgBandN;
	imgBandN = imgdestfinal.clone();
	uchar backgroundColor[3] = { 255,255,255 };
	EraseBackground(&imgdestfinal, &imgBandN, backgroundColor, NULL, 75 / 2, 150 / 2);


	int Long = imgdestfinal.cols;
	int Larg = imgdestfinal.rows;
	int compteur;

	//Affichage Grid Parfaite
	for (int il = 1; il <= 6; il++)
	{
		line(imgdestfinal, Point(Long, ((imgdestfinal.rows / 7) * il)), Point(0, ((imgdestfinal.rows / 7) * il)), Scalar(100, 100, 100), 2, 8, 0);
		for (int ic = 1; ic <= 6; ic++)
		{
			line(imgdestfinal, Point(((imgdestfinal.cols / 7) * ic), Larg), Point(((imgdestfinal.cols / 7) * ic), 0), Scalar(100, 100, 100), 2, 8, 0);
		}
	}


	//Attendre génération image
	waitKey(3000);

	//Image en gray_scale
	cvtColor(imgBandN, imgBandN, COLOR_BGR2GRAY);

	//Initialisation du tableau de struc
	int tab[7][7];

	// Variables boucle
	float nbPRows = imgBandN.rows / 7;
	float nbPCols = imgBandN.cols / 7;


	float pourcentageOccupation;

	for (int il = 0; il < 7; il++)
	{
		for (int ic = 0; ic < 7; ic++)
		{
			compteur = 0;
			// Trouve obstacles
			for (int i = il * nbPRows; i < (il + 1) * nbPRows; i++)
			{
				for (int j = ic * nbPCols; j < (ic + 1) * nbPCols; j++)
				{
					if (imgBandN.at<uchar>(i, j) != 255)
					{
						compteur++;
					}
				}
			}
			//vide
			tab[il][ic] = 3;

			//Obstacles
			pourcentageOccupation = compteur * 100.0f / (float)(nbPCols * nbPRows);		
			if (pourcentageOccupation >= 5)
			{
				tab[il][ic] = 1;

				circle(imgdestfinal, Point((nbPCols * ic) + nbPCols / 2.0f, (nbPRows * il) + nbPRows / 2.0f), 12.0, Scalar(0, 0, 0), -1, 8);
			}
			//Départ
			if (il == 6 && ic == 3)
			{
				tab[il][ic] = 0;
				circle(imgdestfinal, Point((nbPCols * ic) + nbPCols / 2.0f, (nbPRows * il) + nbPRows / 2.0f), 12.0, Scalar(0, 0, 255), -1, 8);
			}
			//Arriver
			if (il == 1 & ic == 2)
			{
				tab[il][ic] = 2;
				circle(imgdestfinal, Point((nbPCols * ic) + nbPCols / 2.0f, (nbPRows * il) + nbPRows / 2.0f), 12.0, Scalar(255, 0, 0), -1, 8);
			}

		}
	}

	NodesGrid nd = NodesGrid(tab, true, imgdestfinal);

	cv::imshow("test", imgBandN);
	cv::waitKey(0);

	return 0;
}