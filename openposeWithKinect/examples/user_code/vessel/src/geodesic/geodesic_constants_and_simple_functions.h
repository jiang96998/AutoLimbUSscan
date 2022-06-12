#ifndef GEODESIC_CONSTANTS
#define GEODESIC_CONSTANTS

// some constants and simple math functions
#include "../tools/Types.h"
namespace geodesic{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

fScalar const GEODESIC_INF = 1e100;

// statistics
fScalar m_time_consumed;		//how much time does the propagation step takes
unsigned m_queue_max_size;			//used for statistics
unsigned m_windows_propagation; // how many time a window is propagated
unsigned m_windows_wavefront; // the number of windows on the wavefront
unsigned m_windows_peak; // the maximum number of windows, used to calculate the memory

// two windows' states after checking
enum windows_state
{
	w1_invalid,
	w2_invalid,
	both_valid
};

inline fScalar cos_from_edges(fScalar const a,			//compute the cosine of the angle given the lengths of the edges
                             fScalar const b,
                             fScalar const c)
{
	assert(a > 1e-50);
	assert(b > 1e-50);
	assert(c > 1e-50);

    fScalar result = (b * b + c * c - a * a)/(2.0 * b * c);
    result = result>-1.0?result:-1.0;
    return result <1.0 ? result:1.0;
}

inline fScalar angle_from_edges(fScalar const a,			//compute the cosine of the angle given the lengths of the edges
                               fScalar const b,
                               fScalar const c)
{
	return acos(cos_from_edges(a, b, c));
}

template<class Points, class Faces>
inline bool read_mesh_from_file(char* filename,
								Points& points,
								Faces& faces, 
								std::vector<int> &realIndex, 
								int& originalVertNum)
{
	std::ifstream file(filename);
	assert(file.is_open());
	if(!file.is_open()) return false;

	char type;
	std::string curLine;
    fScalar coord[3];
	unsigned int vtxIdx[3];
	std::map<std::string, int> mapForDuplicate;
	originalVertNum = 0;

	while(getline(file, curLine))
	{
		if (curLine.size() < 2) continue;
		if (curLine[0] == 'v' && curLine[1] != 't')
		{
			std::map<std::string, int>::iterator pos = mapForDuplicate.find(curLine);
			if (pos == mapForDuplicate.end())
			{
				int oldSize = mapForDuplicate.size();
				realIndex.push_back(oldSize);
				mapForDuplicate[curLine] = oldSize;
                sscanf(curLine.c_str(), "v %lf %lf %lf", &coord[0], &coord[1], &coord[2]);
				for (int i = 0;i < 3;++i) points.push_back(coord[i]);
			}
			else
			{
				realIndex.push_back(pos->second);
			}
			++originalVertNum;
		}
		else if (curLine[0] == 'f')
		{
			unsigned tex;
			if (curLine.find('/') != std::string::npos)
                sscanf(curLine.c_str(), "f %d/%d %d/%d %d/%d", &vtxIdx[0], &tex, &vtxIdx[1], &tex, &vtxIdx[2], &tex);
			else
                sscanf(curLine.c_str(), "f %d %d %d", &vtxIdx[0], &vtxIdx[1], &vtxIdx[2]);
			
			vtxIdx[0] = realIndex[vtxIdx[0]-1];
			vtxIdx[1] = realIndex[vtxIdx[1]-1];
			vtxIdx[2] = realIndex[vtxIdx[2]-1];
			if (vtxIdx[0] == vtxIdx[1] || vtxIdx[0] == vtxIdx[2] || vtxIdx[1] == vtxIdx[2]) continue;

			for (int i = 0;i < 3;++i) faces.push_back(vtxIdx[i]);
		}
	}
	file.close();

	printf("There are %d non-coincidence vertices.\n", points.size() / 3);

	return true;
}



inline void CalculateIntersectionPoint(fScalar X1, fScalar Y1, // compute intersection point of two windows
    fScalar X2, fScalar Y2,
    fScalar X3, fScalar Y3,
    fScalar X4, fScalar Y4,
    fScalar &Intersect_X, fScalar &Intersect_Y)
{
    fScalar A1, B1, C1, A2, B2, C2;
	A1 = Y2 - Y1;
	B1 = X1 - X2;
	C1 = X2 * Y1 - X1 * Y2;
	A2 = Y4 - Y3;
	B2 = X3 - X4;
	C2 = X4 * Y3 - X3 * Y4;

	Intersect_X = (B2 * C1 - B1 * C2) / (A2 * B1 - A1 * B2);
	Intersect_Y = (A1 * C2 - A2 * C1) / (A2 * B1 - A1 * B2);
}


inline bool PointInTriangle(fScalar &X, fScalar &Y, // judge if a point is inside a triangle
    //fScalar Ax, fScalar Ay, // 0, 0
    fScalar &Bx, //fScalar By, // By = 0
    fScalar &Cx, fScalar &Cy)
{
    fScalar dot00 = Cx * Cx + Cy * Cy;// dot00 = dot(v0, v0)
    fScalar dot01 = Cx * Bx;// dot01 = dot(v0, v1)
    fScalar dot02 = Cx * X + Cy * Y;// dot02 = dot(v0, v2)
    fScalar dot11 = Bx * Bx; // dot11 = dot(v1, v1)
    fScalar dot12 = Bx * X;  // dot12 = dot(v1, v2)

    fScalar invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    fScalar u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    fScalar v = (dot00 * dot12 - dot01 * dot02) * invDenom;

	//return (u >= 0) && (v >= 0) && (u + v < 1);
	return (u >= 1e-10) && (v >= 1e-10) && (u + v < 1 - 1e-10);
}


} //geodesic

#endif	//GEODESIC_CONSTANTS_20071231
