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


void
showHelp (char *filename)
{
  std::cout << std::endl;
  std::cout << "***************************************************************************" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "*             Centroid Distance calculation		                     *" << std::endl;
  std::cout << "*                                                                         *" << std::endl;
  std::cout << "***************************************************************************" << std::endl << std::endl;
  std::cout << "Usage: " << filename << " model_csv.csv scene_csv.csv " << std::endl << std::endl;
  std::cout << "Options:" << std::endl;
  std::cout << "     -h:                     Show this help." << std::endl;
  std::cout << "     --t val:         Threshold value - needs to be defined before" << std::endl;

}


int getVectorSize(std::string filename)
{
    std::ifstream input(filename);
    std::string s;
    int j = 0;
    while (std::getline(input , s))
	++j;

   return j;
}


// Get the 3D vector of the saved csv files.
float** get3DVector(int n, std::string filename, bool scene)
{
    std::vector<std::vector<float>> result(n);
    std::ifstream input(filename);
    std::string s;
    float getnumber;
    float** data_matrix = 0;
    data_matrix = new float*[n];
    for (int i = 0; i < n; i++)
    {
        std::getline(input, s);
        std::istringstream iss(s);

        std::string num;
        data_matrix[i] = new float[3];
        int j = 0;
        while (std::getline(iss, num, ','))
            {
           	 getnumber = (float)atof(num.c_str());
           	 std::cout << getnumber << "....: " << i << endl;
           	 data_matrix[i][j] = getnumber;
           	 //result[i].push_back(getnumber);
           	 j++;
            }
    }
  return data_matrix;
}





// Function to calculate 2d distance
float distance2d(float x1, float y1, float x2, float y2)
{
    // Calculating distance
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2) * 1.0);
}
// Function to calculate 3d distance
float distance3d(float x1, float y1,
            float z1, float x2,
            float y2, float z2)
{
    float d = sqrt(pow(x2 - x1, 2) +
                pow(y2 - y1, 2) +
                pow(z2 - z1, 2) * 1.0);
    return d;
}

float rmsValue(float arr[], int n)
{
	float square = 0.0, mean = 0.0, root = 0.0; 
	
	// Calculate square.
	for (int i = 0; i < n; i++) {
		square += pow(arr[i], 2);
	}
	
	// Calculate Mean.
	mean = (square / (float)(n));
	
	// Calculate Root.
	root = sqrt(mean);
	
	return root;
}


int main(int argc, char** argv)
{
	//Show help
	  if (pcl::console::find_switch (argc, argv, "-h"))
	  {
	    showHelp (argv[0]);
	    exit (0);
	  }
	
	float threshold = 0.033;
	pcl::console::parse_argument (argc, argv, "--t", threshold);
	
  	int m = getVectorSize(argv[1]);
  	int s = getVectorSize(argv[2]);
	
	float** m_data = get3DVector(m,argv[1], 0);
	float** s_data = get3DVector(s,argv[2], 1);
	
	float m_dist[m];
	float m_min_index[m];
	
	for (int i = 0; i < m; i++)
	{
		m_dist[i] = 99999999999999999;
		
		for (int j = 0; j < s; j++)
		{
			float d = distance3d(s_data[j][0],s_data[j][1],s_data[j][2],m_data[i][0], m_data[i][1], m_data[i][2]);
			d = abs(d);
			if(d < m_dist[i])
			{
				m_min_index[i] = j;
				m_dist[i] = d;
			}
		}
	}
	
	//Calculating RMS for the whole recognition.
	int n = sizeof(m_dist) / sizeof(m_dist[0]);
	float RMS = rmsValue(m_dist,n);
	
	
	vector<int> m_numbers;
	vector<float> m_distances;
	int number = 0;
	vector<color_array> colors;
	string color[n];
	for (int i = 0; i < m; i++)
	{
		//std::cout << "Closest block to the " << i+1 << " correspondence: " <<  m_min_index[i]+1 << endl;
		//std::cout << "Correspondence distance: " << m_dist[i] << endl;
		m_numbers.push_back(i+1);
		m_distances.push_back(m_dist[i]);
	}
	//std::cout << "Instance number: " << m_numbers.size() << endl;
	//for (auto it = m_distances.begin(); it != m_distances.end(); ++it)
       //cout << ' ' << *it;
       
        for (int i =0; i<m; i++)
        {
        	if (m_dist[i] <= threshold)
        	{
        		number++;
        		colors.push_back({0.4f, 0.5f, 0.5f, 0});
        		color[i] = "g";
        		std::cout << "Below threshold distance [" << i+1 << "] correspondence close to: " <<  m_min_index[i]+1 << endl;
			std::cout << "Correspondence distance: " << m_dist[i] << endl;
        	}
        	else 
        	{
        		colors.push_back({0, 0, 0, 0});
        		color[i] = "r";
        	}

        }
        cout << "Possible recognitions, that are inside the " << threshold << "[m] distance from the block centroid: " << endl;
        cout << number << "/" << m_numbers.size() << endl;
        cout << "RMS value: " << RMS << endl;
        
        std::cout << "Saving absolute distances to file" << std::endl;
	std::ofstream out("centroidDistance.txt");
	for (int i = 0; i < m_distances.size (); i++)
	{
		out << m_dist[i]*100;
		out << "\n";
	}
	out.close();
        
        
        bar(m_distances, 0.5);//->face_color(colors);
        
        
   

        ylabel("Distance from the nearest block");
        xlabel("Instance number");
        hold(on);

        
        
        std::vector<double> X = linspace(0, m_numbers[m-1], 50);
        //std::vector<double> th;
        std::vector<double> Y = transform(X, [](double x) { return 0.033; });
        plot(X, Y)->line_width(2).color("red");

        auto l = legend("Distances", "Threshold value");
	l->location(legend::general_alignment::topleft);
	
        show();
        


    	return 0;
}






