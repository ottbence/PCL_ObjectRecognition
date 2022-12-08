#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/centroid.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <matplot/matplot.h>
#include <cmath>
#include <random>
#include <set>

using namespace std;
using namespace matplot;

int getVectorSize(std::string filename)
{
    std::ifstream input(filename);
    std::string s;
    int j = 0;
    while (std::getline(input , s))
	++j;

   return j;
}

float* getVector(int n, std::string filename)
{
    std::vector<std::vector<float>> result(n);
    std::ifstream input(filename);
    std::string s;
    float getnumber;
    float* data_matrix = 0;
    data_matrix = new float[n];
    for (int i = 0; i < n; i++)
    {
        std::getline(input, s);
        std::istringstream iss(s);
        std::string num;

        while (std::getline(iss, num))
            {
           	 getnumber = (float)atof(num.c_str());
           	 data_matrix[i] = getnumber;
            }
    }
  return data_matrix;
}
int
main(int argc, char** argv)
{

	
	//
	// D435i sb 848p
	//
	/*
	int m1 = getVectorSize("Data/centroidDistance[22].txt");
	int m2 = getVectorSize("Data/centroidDistance[21].txt");
	int m3 = getVectorSize("Data/centroidDistance[20].txt");

	
	float* m1_data = getVector(m1,"Data/centroidDistance[22].txt");
	float* m2_data = getVector(m2,"Data/centroidDistance[21].txt");
	float* m3_data = getVector(m3,"Data/centroidDistance[20].txt");

	
	int sum = m1+m2+m3;
	vector<double> blockdistance;
	vector<string> framedistance;
	
	for (int i=0; i<m1; i++)
	{
		blockdistance.push_back(m1_data[i]);
		framedistance.push_back("31 cm");
	}
	for (int i=0; i<m2; i++)
	{
		blockdistance.push_back(m2_data[i]);
		framedistance.push_back("35 cm");
	}
	for (int i=0; i<m3; i++)
	{
		blockdistance.push_back(m3_data[i]);
		framedistance.push_back("38 cm");
	}

	boxplot(blockdistance, framedistance);
	xlabel("A tábla és a kamera közti távolság [cm]");
    	ylabel("Abszolút távolság a legközelebbi blokk középpontjától [cm]");
    	title("D435i 1280x720p Kis méretű tábla - A detektálások távolsága a legközelebbi blokk középpontjától");
	show();*/
	
	
	
	
	
	
	//
	// D435i sb 848p
	//
	/*
	int m1 = getVectorSize("Data/centroidDistance[16].txt");
	int m2 = getVectorSize("Data/centroidDistance[17].txt");
	int m3 = getVectorSize("Data/centroidDistance[18].txt");
	int m4 = getVectorSize("Data/centroidDistance[19].txt");
	
	float* m1_data = getVector(m1,"Data/centroidDistance[16].txt");
	float* m2_data = getVector(m2,"Data/centroidDistance[17].txt");
	float* m3_data = getVector(m3,"Data/centroidDistance[18].txt");
	float* m4_data = getVector(m4,"Data/centroidDistance[19].txt");
	
	int sum = m1+m2+m3+m4;
	vector<double> blockdistance;
	vector<string> framedistance;
	
	for (int i=0; i<m1; i++)
	{
		blockdistance.push_back(m1_data[i]);
		framedistance.push_back("31 cm");
	}
	for (int i=0; i<m2; i++)
	{
		blockdistance.push_back(m2_data[i]);
		framedistance.push_back("33 cm");
	}
	for (int i=0; i<m3; i++)
	{
		blockdistance.push_back(m3_data[i]);
		framedistance.push_back("34 cm");
	}
	for (int i=0; i<m4; i++)
	{
		blockdistance.push_back(m4_data[i]);
		framedistance.push_back("35 cm");
	}
	boxplot(blockdistance, framedistance);
	xlabel("A tábla és a kamera közti távolság [cm]");
    	ylabel("Abszolút távolság a legközelebbi blokk középpontjától [cm]");
    	title("D435i 848x480p Kis méretű tábla - A detektálások távolsága a legközelebbi blokk középpontjától");
	show();
	
	*/
	
	
	
	//
	// D435i sb 640p
	//
	/*
	int m1 = getVectorSize("Data/centroidDistance[13].txt");
	int m2 = getVectorSize("Data/centroidDistance[14].txt");
	int m3 = getVectorSize("Data/centroidDistance[15].txt");

	
	float* m1_data = getVector(m1,"Data/centroidDistance[13].txt");
	float* m2_data = getVector(m2,"Data/centroidDistance[14].txt");
	float* m3_data = getVector(m3,"Data/centroidDistance[15].txt");

	int sum = m1+m2+m3;
	vector<double> blockdistance;
	vector<string> framedistance;
	
	for (int i=0; i<m1; i++)
	{
		blockdistance.push_back(m1_data[i]);
		framedistance.push_back("28 cm [1]");
	}
	for (int i=0; i<m2; i++)
	{
		blockdistance.push_back(m2_data[i]);
		framedistance.push_back("28 cm [2]");
	}
	for (int i=0; i<m3; i++)
	{
		blockdistance.push_back(m3_data[i]);
		framedistance.push_back("28 cm [3]");
	}

	boxplot(blockdistance, framedistance);
	xlabel("A tábla és a kamera közti távolság [cm]");
    	ylabel("Abszolút távolság a legközelebbi blokk középpontjától [cm]");
    	title("D435i 640x360p Kis méretű tábla - A detektálások távolsága a legközelebbi blokk középpontjától");
	show();*/
	
	
	
	
	
	//
	// D455 1280 bb
	//
	/*
	int m1 = getVectorSize("Data/centroidDistance[10].txt");
	int m2 = getVectorSize("Data/centroidDistance[12].txt");
	int m3 = getVectorSize("Data/centroidDistance[26].txt");
	int m4 = getVectorSize("Data/centroidDistance[8].txt");
	int m5 = getVectorSize("Data/centroidDistance[9].txt");
	
	float* m1_data = getVector(m1,"Data/centroidDistance[10].txt");
	float* m2_data = getVector(m2,"Data/centroidDistance[12].txt");
	float* m3_data = getVector(m3,"Data/centroidDistance[26].txt");
	float* m4_data = getVector(m4,"Data/centroidDistance[8].txt");
	float* m5_data = getVector(m5,"Data/centroidDistance[9].txt");
	
	int sum = m1+m2+m3+m4+m5;
	vector<double> blockdistance;
	vector<string> framedistance;
	
	for (int i=0; i<m1; i++)
	{
		blockdistance.push_back(m1_data[i]);
		framedistance.push_back("52 cm");
	}
	for (int i=0; i<m2; i++)
	{
		blockdistance.push_back(m2_data[i]);
		framedistance.push_back("53 cm");
	}
	for (int i=0; i<m3; i++)
	{
		blockdistance.push_back(m3_data[i]);
		framedistance.push_back("60 cm");
	}
	for (int i=0; i<m4; i++)
	{
		blockdistance.push_back(m4_data[i]);
		framedistance.push_back("71 cm");
	}
	for (int i=0; i<m5; i++)
	{
		blockdistance.push_back(m5_data[i]);
		framedistance.push_back("75 cm");
	}
	
	boxplot(blockdistance, framedistance);
	xlabel("A tábla és a kamera közti távolság [cm]");
    	ylabel("Abszolút távolság a legközelebbi blokk középpontjától [cm]");
    	title("D455 1280x720p Nagy méretű tábla - A detektálások távolsága a legközelebbi blokk középpontjától");
	show();*/
	
	//
	// D435i 1280 bb
	//
	/*
	int m1 = getVectorSize("Data/centroidDistance[4].txt");
	int m2 = getVectorSize("Data/centroidDistance[3].txt");
	int m3 = getVectorSize("Data/centroidDistance[5].txt");
	int m4 = getVectorSize("Data/centroidDistance[28].txt");
	int m5 = getVectorSize("Data/centroidDistance[6].txt");
	int m6 = getVectorSize("Data/centroidDistance[7].txt");
	
	float* m1_data = getVector(m1,"Data/centroidDistance[4].txt");
	float* m2_data = getVector(m2,"Data/centroidDistance[3].txt");
	float* m3_data = getVector(m3,"Data/centroidDistance[5].txt");
	float* m4_data = getVector(m4,"Data/centroidDistance[28].txt");
	float* m5_data = getVector(m5,"Data/centroidDistance[6].txt");
	float* m6_data = getVector(m6,"Data/centroidDistance[7].txt");
	
	int sum = m1+m2+m3+m4+m5;
	vector<double> blockdistance;
	vector<string> framedistance;
	
	for (int i=0; i<m1; i++)
	{
		blockdistance.push_back(m1_data[i]);
		framedistance.push_back("32 cm");
	}
	for (int i=0; i<m2; i++)
	{
		blockdistance.push_back(m2_data[i]);
		framedistance.push_back("35 cm");
	}
	for (int i=0; i<m3; i++)
	{
		blockdistance.push_back(m3_data[i]);
		framedistance.push_back("39 cm");
	}
	for (int i=0; i<m4; i++)
	{
		blockdistance.push_back(m4_data[i]);
		framedistance.push_back("45 cm");
	}
	for (int i=0; i<m5; i++)
	{
		blockdistance.push_back(m5_data[i]);
		framedistance.push_back("54 cm");
	}
	for (int i=0; i<m6; i++)
	{
		blockdistance.push_back(m6_data[i]);
		framedistance.push_back("73 cm");
	}
	
	boxplot(blockdistance, framedistance);
	xlabel("A tábla és a kamera közti távolság [cm]");
    	ylabel("Abszolút távolság a legközelebbi blokk középpontjától [cm]");
    	title("D435i 1280x720p Nagy méretű tábla - A detektálások távolsága a legközelebbi blokk középpontjától");
	show();
	*/
	//
	//0. fázis distances boxplot
	//
	/*
	
	int m1 = getVectorSize("Data/centroidDistance_0_1_mm.txt");
	int m2 = getVectorSize("Data/centroidDistance_0_2_mm.txt");
	int m3 = getVectorSize("Data/centroidDistance_0_3_mm.txt");
	int m4 = getVectorSize("Data/centroidDistance_0_4_mm.txt");

	
	float* m1_data = getVector(m1,"Data/centroidDistance_0_1_mm.txt");
	float* m2_data = getVector(m2,"Data/centroidDistance_0_2_mm.txt");
	float* m3_data = getVector(m3,"Data/centroidDistance_0_3_mm.txt");
	float* m4_data = getVector(m4,"Data/centroidDistance_0_4_mm.txt");


	vector<double> blockdistance;
	vector<string> framedistance;
	
	for (int i=0; i<m1; i++)
	{
		blockdistance.push_back(m1_data[i]);
		framedistance.push_back("SHOT,GC");
	}
	for (int i=0; i<m2; i++)
	{
		blockdistance.push_back(m2_data[i]);
		framedistance.push_back("SHOT,Hough");
	}
	for (int i=0; i<m4; i++)
	{
		blockdistance.push_back(m4_data[i]);
		framedistance.push_back("FPFH,GC");
	}
	for (int i=0; i<m3; i++)
	{
		blockdistance.push_back(m3_data[i]);
		framedistance.push_back("FPFH,Hough");
	}

	
	boxplot(blockdistance, framedistance);
	xlabel("Felhasznált deszkriptor, klaszterezési algoritmus");
    	ylabel("Abszolút távolság a legközelebbi blokk középpontjától [mm]");
    	title("Nulladik fázis - A felhasznált deszkriptorok és klaszterezési algoritmusok tesztelése");
	show();*/
	
	//
	// Descriptor tesztelés valós jeleneten.
	//
	
	int m1 = getVectorSize("Data/centroidDistance[24].txt");
	int m2 = getVectorSize("Data/centroidDistance[25].txt");
	int m3 = getVectorSize("Data/centroidDistance[23].txt");
	int m4 = getVectorSize("Data/centroidDistance[3].txt");

	
	float* m1_data = getVector(m1,"Data/centroidDistance[24].txt");
	float* m2_data = getVector(m2,"Data/centroidDistance[25].txt");
	float* m3_data = getVector(m3,"Data/centroidDistance[23].txt");
	float* m4_data = getVector(m4,"Data/centroidDistance[3].txt");


	vector<double> blockdistance;
	vector<string> framedistance;
	
	for (int i=0; i<m1; i++)
	{
		blockdistance.push_back(m1_data[i]);
		framedistance.push_back("SHOT,GC");
	}
	for (int i=0; i<m2; i++)
	{
		blockdistance.push_back(m2_data[i]);
		framedistance.push_back("SHOT,Hough");
	}
	for (int i=0; i<m4; i++)
	{
		blockdistance.push_back(m4_data[i]);
		framedistance.push_back("FPFH,GC");
	}
	for (int i=0; i<m3; i++)
	{
		blockdistance.push_back(m3_data[i]);
		framedistance.push_back("FPFH,Hough");
	}

	
	boxplot(blockdistance, framedistance);
	xlabel("Felhasznált deszkriptor, klaszterezési algoritmus");
    	ylabel("Abszolút távolság a legközelebbi blokk középpontjától [cm]");
    	title("Nulladik fázis - A felhasznált deszkriptorok és klaszterezési algoritmusok tesztelése valós jeleneten");
	show();
	
	// 0.fázis RMS 
	/*vector<double> blockdistance = {0.01815, 0.0159855, 0.0142701, 0.0181818};
	 bar(blockdistance);

	gca()->x_axis().ticklabels({"SHOT,GC", "SHOT,Hough", "FPFH,GC", "FPFH,Hough"});
	xlabel("Used descriptor, clustering algorithm");
    	ylabel("RMS of the absolute distances");
    	title("Zero phase testing the algorithm");
	show();*/
}	
