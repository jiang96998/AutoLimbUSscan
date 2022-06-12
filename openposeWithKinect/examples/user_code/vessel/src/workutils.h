#ifndef WORKUTILS
#define WORKUTILS
#include <tools/Types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/obj_io.h>
#include <json/reader.h>

//vtk
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkProperty.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkSphereSource.h>
#include <vtkGlyph3DMapper.h>
#include <fstream>
#include <iostream>
#include <vtkKochanekSpline.h>
#include <vtkPolyDataMapper.h>
#include <vtkBuffer.h>
#include <vtkPolyDataWriter.h>

class workutils
{
private:
    /* data */
public:
    workutils(/* args */);
    ~workutils();
    static void LocateVesselOnSurface();
    static MatrixXX loadPointsfromTxt(std::string filename);
    static void LocateTrajectoryOnSurface(int isUpperArm);
    static bool read_data(const std::string filename, Mesh& mesh);
    static void calRotation(Eigen::Vector3f u, Eigen::Vector3f v, double &angle, Eigen::Vector3f &vec);
    static Eigen::Matrix4f RodriguesMatrixTranslation(Eigen::Vector3f n, double angle);
    static Eigen::Matrix4f initialAlign(std::string PLYfilename,int isUpperArm);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudFromText(std::string filename);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr getCloudFromPLY(std::string filename);
    static bool fitSpline();
    static void transformSurface();
};

pcl::PointCloud<pcl::PointXYZ>::Ptr workutils::getCloudFromText(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(filename);
    assert(file.is_open());
    std::string line;
    
    while (std::getline(file,line))
    {
        pcl::PointXYZ point;
        std::stringstream ss(line);
        ss>>point.x>>point.y>>point.z;
        cloud->push_back(point);
    }
    return cloud;
}
pcl::PointCloud<pcl::PointXYZ>::Ptr workutils::getCloudFromPLY(std::string filename)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::ifstream file(filename);
    assert(file.is_open());
    std::string line;
    for (size_t i = 0; i < 10; i++)
    {
        std::getline(file,line);
    }
    
    
    while (std::getline(file,line))
    {
        pcl::PointXYZ point;
        std::stringstream ss(line);
        ss>>point.x>>point.y>>point.z;
        cloud->push_back(point);
    }
    return cloud;
}

void workutils::LocateVesselOnSurface()
{
    OpenMesh::IO::Options opt_read = OpenMesh::IO::Options::VertexNormal;
    OpenMesh::TriMesh_ArrayKernelT<> src;
    OpenMesh::IO::read_mesh(src,"6.obj");
    int n = src.n_vertices();
    std::cout<<"n::"<<n<<std::endl;
    MatrixXX src_points;
    //将tar_points_设置为3*n_t的矩阵
    src_points.resize(3, n);
    //将target顶点存入tar_points_中
    for (int i = 0; i < n; i++)
    {
        src_points(0, i) = src.point(src.vertex_handle(i))[0];
        src_points(1, i) = src.point(src.vertex_handle(i))[1];
        src_points(2, i) = src.point(src.vertex_handle(i))[2];
    }
    KDtree* srctree=new KDtree(src_points);
    Json::Value root;
    Json::Reader reader;
    std::ifstream ifs("Centerline curve (0).mrk.json");
    if(!reader.parse(ifs,root))
    {
        std::cout<<"fail to open json"<<std::endl;
    }
    else
    {
        std::cout<<"successful to open json"<<std::endl;
        // const Json::Value markups=root["markups"];
        const Json::Value markups = root["markups"];
        auto arr1=markups[0];
        auto points=arr1["controlPoints"];
        std::vector<float*> selectpoint;
        std::cout<<points.size()<<std::endl;
        for (int i = 0; i < points.size(); i++)
        {
            float* p=new float[3];
            p[0]=points[i]["position"][0].asFloat();
            p[1]=points[i]["position"][1].asFloat();
            p[2]=points[i]["position"][2].asFloat();
            selectpoint.push_back(p);
        }
        std::ofstream file4("vessel4.txt");
        for (size_t i = 0; i < points.size(); i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                file4<<selectpoint[i][j]<<" ";
            }
            file4<<std::endl;
        }

        std::vector<float*> srcpoints;
        for (int i = 0; i < points.size(); i++)
        {
            fScalar mini_dist;
            //通过KDtree寻找距离source mesh点对应的最近的target上的点，并将最小距离存入mini_dist

            int idx = srctree->closest(selectpoint[i], mini_dist);
            std::cout<<"idx::::"<<idx<<std::endl;
            std::cout<<"Minidist::::"<<mini_dist<<std::endl;
            srcpoints.push_back(src_points.col(idx).data());   
                  
        }
        std::cout<<src.point(src.vertex_handle(1)).data()[0]<<std::endl;
        std::ofstream file3("vessel5.txt");
        for (size_t i = 0; i < points.size(); i++)
        {
            for (size_t j = 0; j < 3; j++)
            {
                file3<<srcpoints[i][j]<<" ";
                // file3<<src_points(j,i)<<" ";
                // file3<<src.point(src.vertex_handle(i)).data()[j]<<" ";
            }
            file3<<std::endl;
        }
        std::cout<<"successful"<<std::endl;
    }
}

void workutils::LocateTrajectoryOnSurface(int isUpperArm)
{ 
    MatrixXX src_points;
    std::string fileNamesrc;
    std::string fileNametar;
    std::string fileNameRes;
    std::string fileNameVessel;
    if(isUpperArm==1)
    {
        fileNamesrc="./result/Uppersource.txt";
        fileNametar="./result/Uppertarget.txt";
        fileNameRes="./result/UpperResult.txt";
        fileNameVessel="./template/UpperVessel.xyz";
    }
    else
    {
        fileNamesrc="./result/Lowersource.txt";
        fileNametar="./result/Lowertarget.txt";
        fileNameRes="./result/LowerResult.txt";
        fileNameVessel="./template/LowerVessel.xyz";
    }
    src_points=loadPointsfromTxt(fileNamesrc);
    KDtree* srctree=new KDtree(src_points);

    MatrixXX vel_points;
    vel_points=loadPointsfromTxt(fileNameVessel);
    float *p=new float[3];
    std::vector<int> srcpoints;
        for (int i = 0; i < vel_points.cols(); i++)
        {
            fScalar mini_dist;
            //通过KDtree寻找距离source mesh点对应的最近的target上的点，并将最小距离存入mini_dist
            for (size_t j = 0; j < 3; j++)
            {
                p[j]=vel_points(j,i);
            }
            
            int idx = srctree->closest(p, mini_dist);
            // std::cout<<"idx::::"<<idx<<std::endl;
            // std::cout<<"Minidist::::"<<mini_dist<<std::endl;
            // srcpoints.push_back(src_points.col(idx).data());  
            srcpoints.push_back(idx);                  
        }

    // std::cout<<srcpoints.size()<<" gggggggggggggggggg"<<std::endl;
    MatrixXX tar_points;
    tar_points=loadPointsfromTxt(fileNametar);

    std::ofstream file(fileNameRes);
        for (size_t i = 0; i < srcpoints.size(); i++)
        {
            // file3<<srcpoints[i]<<std::endl;
            for (size_t j = 0; j < 3; j++)
            {
                file<<tar_points(j,srcpoints[i])<<" ";
            }
            file<<std::endl;
        }
    std::cout<<"successfullllll"<<std::endl;
}

MatrixXX workutils::loadPointsfromTxt(std::string filename)
{
    std::ifstream pointsFile(filename);
	assert(pointsFile.is_open());
	std::string line;
    MatrixXX points;
    int n=0;
    while (std::getline(pointsFile, line))
	{
        n++;
	}
    pointsFile.close();
    std::ifstream pointsFile2(filename);
    points.resize(3,n);
    int j=0;
	while (std::getline(pointsFile2, line))
	{
		std::stringstream ss(line);
		for (size_t i = 0; i<3; i++)
		{
			ss >> points(i, j);
		}
        j++;
	}

    return points;
}


bool workutils::read_data(const std::string filename, Mesh& mesh)
{
    OpenMesh::IO::Options opt_read = OpenMesh::IO::Options::VertexNormal;
    mesh.request_vertex_normals();
    bool read_OK = OpenMesh::IO::read_mesh(mesh, filename,opt_read);

	std::cout << "filename = " << filename << std::endl;
    if (read_OK)
    {
        mesh.request_vertex_status();
        mesh.request_edge_status();
        mesh.request_face_status();

        mesh.request_face_normals();
        // printBasicMeshInfo(mesh);

        mesh.update_face_normals();
        if(mesh.n_faces()>0)
            mesh.update_vertex_normals();

        Vec3 MeshScales;
        MeshScales[0] = 0.0; MeshScales[1] = 0.0; MeshScales[2] = 0.0;
        for (Mesh::VertexIter v_it = mesh.vertices_begin(); v_it != mesh.vertices_end(); ++v_it)
        {
            MeshScales += mesh.point(*v_it);
        }
        MeshScales /= mesh.n_vertices();
        return true;
    }
    std::cout << "#vertices = " << mesh.n_vertices() << std::endl;
    return false;
}

void workutils::calRotation(Eigen::Vector3f u, Eigen::Vector3f v, double &angle, Eigen::Vector3f &vec)
{
	angle = acos(u.dot(v) / (u.norm()*v.norm()));
    std::cout<<angle<<std::endl;
	// if (angle > M_PI / 2)
	// {
	// 	u = -u;
	// 	angle = M_PI - angle;
	// }
	float i, j, k;
	i = u(1)*v(2) - u(2)*v(1);
    j = v(0)*u(2) - v(2)*u(0);
    k = u(0)*v(1) - u(1)*v(0);
	vec << i, j, k;
	double value = sqrt(i*i + j*j + k*k);
	vec(0) = vec(0) / value;
	vec(1) = vec(1) / value;
	vec(2) = vec(2) / value;
}

Eigen::Matrix4f workutils::RodriguesMatrixTranslation(Eigen::Vector3f n, double angle)
{
	//罗德里格斯公式求旋转矩阵
	Eigen::Matrix4f x_transform(Eigen::Matrix4f::Identity());
	x_transform(0, 0) = cos(angle) + n(0)*n(0)*(1 - cos(angle));
	x_transform(1, 0) = n(2)*sin(angle) + n(0)*n(1)*(1 - cos(angle));
	x_transform(2, 0) = -n(1)*sin(angle) + n(0)*n(2)*(1 - cos(angle));
	x_transform(0, 1) = n(0)*n(1)*(1 - cos(angle)) - n(2)*sin(angle);
	x_transform(1, 1) = cos(angle) + n(1)*n(1)*(1 - cos(angle));
	x_transform(2, 1) = n(0)*sin(angle) + n(1)*n(2)*(1 - cos(angle));
	x_transform(0, 2) = n(1)*sin(angle) + n(0)*n(2)*(1 - cos(angle));
	x_transform(1, 2) = -n(0)*sin(angle) + n(1)*n(2)*(1 - cos(angle));
	x_transform(2, 2) = cos(angle) + n(2)*n(2)*(1 - cos(angle));

	return  x_transform;
}

Eigen::Matrix4f workutils::initialAlign(std::string PLYfilename,int isUpperArm)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    std::ifstream file1("./template/sourcefeature.txt");
    std::string line;
    Eigen::Matrix3f psrc;
    for (size_t i = 0; i < 3; i++)
    {
        std::getline(file1,line);
        std::stringstream ss(line);
        for (size_t j = 0; j < 3; j++)
        {
            ss>>psrc(j,i);
        } 
    }

    std::ifstream file2("./result/featureXYZ.txt");
    Eigen::Matrix3f ptar;
    for (size_t i = 0; i < 3; i++)
    {
        std::getline(file2,line);
        std::stringstream ss(line);
        for (size_t j = 0; j < 3; j++)
        {
            ss>>ptar(j,i);
        } 
    }
    Eigen::Matrix3Xf vsrc(3,2);
    Eigen::Matrix3Xf vtar(3,2);
    for (size_t i = 0; i < 2; i++)
    {
        for (size_t j = 0; j < 3; j++)
        {
            vsrc(j,i)=psrc(j,i+1)-psrc(j,i);
            vtar(j,i)=ptar(j,i+1)-ptar(j,i);
        }        
    }

	pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>());//源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>());//目标点云
    // pcl::io::loadPLYFile(PLYfilename,*target);
    target=getCloudFromPLY(PLYfilename);

    pcl::PointCloud<pcl::PointXYZ>::Ptr target_t(new pcl::PointCloud<pcl::PointXYZ>);
	//平移目标点云，将目标点云通过质心平移到原点
	Eigen::Matrix4f translation_t = Eigen::Matrix4f::Identity();
	//设置矩阵的元素
	translation_t(0, 3) = -ptar(0,2-isUpperArm);
	translation_t(1, 3) = -ptar(1,2-isUpperArm);
	translation_t(2, 3) = -ptar(2,2-isUpperArm);
	pcl::transformPointCloud(*target, *target_t, translation_t);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr source_t(new pcl::PointCloud<pcl::PointXYZ>);
	//平移源点云，将源点云通过质心平移到原点
	Eigen::Matrix4f translation_s = Eigen::Matrix4f::Identity();
	//设置矩阵的元素
	translation_s(0, 3) = psrc(0,2-isUpperArm);
	translation_s(1, 3) = psrc(1,2-isUpperArm);
	translation_s(2, 3) = psrc(2,2-isUpperArm);
	// pcl::transformPointCloud(*source, *source_t, translation_s);

	pcl::PointCloud<pcl::PointXYZ>::Ptr translationCloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector3f n0, n1;
	double angle0, angle1;
	calRotation(vtar.col(0),vsrc.col(0),  angle0, n0);
	calRotation(vtar.col(1),vsrc.col(1),  angle1, n1);
	Eigen::Matrix4f transform0(Eigen::Matrix4f::Identity());
	Eigen::Matrix4f transform1(Eigen::Matrix4f::Identity());
    std::vector<Eigen::Matrix4f> vM;
	transform0 = RodriguesMatrixTranslation(n0, angle0);
	transform1 = RodriguesMatrixTranslation(n1, angle1);
    vM.push_back(transform1);
    vM.push_back(transform0);
	pcl::transformPointCloud(*target_t, *translationCloud, vM[isUpperArm]);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::transformPointCloud(*translationCloud, *transformedCloud, translation_s);
    pcl::PLYWriter plywriter;
    std::string fileName;
    if(isUpperArm==1)
    {
        fileName="./result/transformedTargetUpper.ply";
    }
    else
    {
        fileName="./result/transformedTargetLower.ply";
    }
    plywriter.write(fileName,*transformedCloud);
    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    transform=translation_s*vM[isUpperArm]*translation_t;
    return transform;
    // pcl::visualization::PCLVisualizer viewer("Viewer");
    // viewer.initCameraParameters();
    // viewer.setBackgroundColor(0.1,0.1,0.1);
	// // viewer.addCoordinateSystem(0.5);
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_v2(transformedCloud, 255, 10, 0);
    // viewer.addPointCloud<pcl::PointXYZ>(transformedCloud,color_v2,"cc");	
    // while (!viewer.wasStopped())
	// {
	// 	viewer.spinOnce(100);
	// }
}

bool workutils::fitSpline()
{
    vtkSmartPointer<vtkKochanekSpline> xSpline=vtkSmartPointer<vtkKochanekSpline>::New();
    vtkSmartPointer<vtkKochanekSpline> ySpline=vtkSmartPointer<vtkKochanekSpline>::New();
    vtkSmartPointer<vtkKochanekSpline> zSpline=vtkSmartPointer<vtkKochanekSpline>::New();

    vtkSmartPointer<vtkPoints> points =
        vtkSmartPointer<vtkPoints>::New();

    std::fstream file1("./result/UpperResult.txt");
    std::string line;
    std::stringstream ss;
    while (getline(file1, line))
    {
        ss.clear();
        ss.str(line);
        double p[3] = {0, 0, 0};
        ss >> p[0] >> p[1] >> p[2];
        points->InsertNextPoint(p);
    }    
    std::fstream file2("./result/LowerResult.txt");
    while (getline(file2, line))
    {
        ss.clear();
        ss.str(line);
        double p[3] = {0, 0, 0};
        ss >> p[0] >> p[1] >> p[2];
        points->InsertNextPoint(p);
    }

    // points->InsertNextPoint(p0);
    // points->InsertNextPoint(p1);
    // points->InsertNextPoint(p2);
    // points->InsertNextPoint(p3);

    vtkSmartPointer<vtkParametricSpline> spline =
        vtkSmartPointer<vtkParametricSpline>::New();
    spline->SetXSpline(xSpline);
    spline->SetYSpline(ySpline);
    spline->SetZSpline(zSpline);
    spline->SetPoints(points);
    spline->ClosedOff();

    vtkSmartPointer<vtkParametricFunctionSource> functionSource =
        vtkSmartPointer<vtkParametricFunctionSource>::New();
    functionSource->SetParametricFunction(spline);
    functionSource->SetUResolution(60);
    functionSource->SetVResolution(10000);
    functionSource->SetWResolution(10000);
    functionSource->Update();

    //保存数据
	auto Writer = vtkSmartPointer<vtkPolyDataWriter>::New();
	Writer->SetFileName("./result/spline.vtk");
	Writer->SetInputData(functionSource->GetOutput());//vtkPolyData
	Writer->Write();

    // vtkSmartPointer<vtkPolyDataMapper> splineMapper =vtkSmartPointer<vtkPolyDataMapper>::New();
    // splineMapper->SetInputConnection(functionSource->GetOutputPort());

    // vtkSmartPointer<vtkActor> splineActor =
    //     vtkSmartPointer<vtkActor>::New();
    // splineActor->SetMapper(splineMapper);

    // vtkSmartPointer<vtkSphereSource> sphereSource =
    //     vtkSmartPointer<vtkSphereSource>::New();
    // sphereSource->SetPhiResolution(21);
    // sphereSource->SetThetaResolution(21);
    // sphereSource->SetRadius(.1);

    // vtkSmartPointer<vtkPolyData> splinePointsData =
    //     vtkSmartPointer<vtkPolyData>::New();
    // splinePointsData->SetPoints(points);

    // vtkSmartPointer<vtkGlyph3DMapper> splinePointsMapper =
    //     vtkSmartPointer<vtkGlyph3DMapper>::New();
    // splinePointsMapper->SetInputData(splinePointsData);
    // splinePointsMapper->SetSourceConnection(sphereSource->GetOutputPort());

    // vtkSmartPointer<vtkActor> pointsActor =
    //     vtkSmartPointer<vtkActor>::New();
    // pointsActor->SetMapper(splinePointsMapper);
    // pointsActor->GetProperty()->SetColor(1, 0, 0);

    // vtkSmartPointer<vtkRenderer> renderer =
    //     vtkSmartPointer<vtkRenderer>::New();
    // vtkSmartPointer<vtkRenderWindow> renderWindow =
    //     vtkSmartPointer<vtkRenderWindow>::New();
    // renderWindow->SetSize(600, 600);
    // renderWindow->AddRenderer(renderer);
    // vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    //     vtkSmartPointer<vtkRenderWindowInteractor>::New();
    // renderWindowInteractor->SetRenderWindow(renderWindow);

    // renderer->AddActor(splineActor);
    // renderer->AddActor(pointsActor);

    // renderWindow->Render();
    // renderWindowInteractor->Start();

    std::fstream vtkfile("./result/spline.vtk");
    for (size_t i = 0; i < 5; i++)
    {
        getline(vtkfile,line);
    }
    std::ofstream txtfile("./result/whole_vessel_trajectory.txt");
    std::vector<float> Vpoints;
    for (size_t i = 0; i < 20; i++)
    {
        getline(vtkfile,line);
        ss.clear();
        ss.str(line);
        float p=0;
        while (ss>>p)
        {
            Vpoints.push_back(p);
        }
    }
    std::vector<std::vector<float>> V3points;
    for (size_t i = 0; i < Vpoints.size(); i+=3)
    {
        std::vector<float> point;
        point.push_back(Vpoints[i]);
        point.push_back(Vpoints[i+1]);
        point.push_back(Vpoints[i+2]);
        V3points.push_back(point);
    }
    // std::reverse(V3points.begin(), V3points.end());

    for (size_t i = 0; i < V3points.size(); i++)
    {
        txtfile<<V3points[i][0]<<" "<<V3points[i][1]<<" "<<V3points[i][2]<<std::endl;
    }
    
    // for (size_t i = 0; i < Vpoints.size(); i+=3)
    // {
    //     txtfile<<Vpoints[i]<<" "<<Vpoints[i+1]<<" "<<Vpoints[i+2]<<std::endl;
    // } 
    return true;
}

workutils::workutils(/* args */)
{
}

workutils::~workutils()
{
}



#endif