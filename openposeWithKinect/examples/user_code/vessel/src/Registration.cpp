#include "Registration.h"
#include <median.h>
#include <iostream>
#include <unordered_map>
// #include <workutils.h>

#if (__cplusplus >= 201402L) || (defined(_MSC_VER) && _MSC_VER >= 1800)
#define MAKE_UNIQUE std::make_unique
#else
#define MAKE_UNIQUE company::make_unique
#endif

Registration::Registration() {
    target_tree = NULL;
    src_mesh_ = NULL;
    tar_mesh_ = NULL;
    use_cholesky_solver_ = true;
    use_pardiso_ = true;
    update_tarGeotree = true;
};

Registration::~Registration()
{
    if (target_tree != NULL)
    {
        delete target_tree;
        target_tree = NULL;
    }
}

// initialize before rigid transformation
void Registration::rigid_init(Mesh & src_mesh, Mesh & tar_mesh, RegParas& paras)
{
    //src_mesh为源模型
    src_mesh_ = new Mesh;
    //tar_mesh为目标模型
    tar_mesh_ = new Mesh;

    src_mesh_ = &src_mesh;
    tar_mesh_ = &tar_mesh;
    //pars_为参数集
    pars_ = paras;
    //输入mesh文件，获取顶点
    n_src_vertex_ = src_mesh_->n_vertices();
    n_tar_vertex_ = tar_mesh_->n_vertices();
    
    //将corres_pair_ids_设置为源模型顶点个数的n_s行数
    corres_pair_ids_.resize(n_src_vertex_);

    //将tar_points_设置为3*n_t的矩阵
    tar_points_.resize(3, n_tar_vertex_);

    #pragma omp parallel for
    //将target顶点存入tar_points_中
    for (int i = 0; i < n_tar_vertex_; i++)
    {
        tar_points_(0, i) = tar_mesh_->point(tar_mesh_->vertex_handle(i))[0];
        tar_points_(1, i) = tar_mesh_->point(tar_mesh_->vertex_handle(i))[1];
        tar_points_(2, i) = tar_mesh_->point(tar_mesh_->vertex_handle(i))[2];
    }

    // construct kd Tree
    target_tree = new KDtree(tar_points_);
    InitCorrespondence(correspondence_pairs_);
    std::cout<<"刚性配准初始点对数："<<correspondence_pairs_.size()<<std::endl;
}

/// Find self edge median of point cloud
template<typename Derived1>
fScalar Registration::FindKnearestMed(Eigen::MatrixBase<Derived1>& X, int nk)
{
    nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived1>, 3, nanoflann::metric_L2_Simple> kdtree(X);
    VectorX X_nearest(X.cols());
#pragma omp parallel for
    for (int i = 0; i<X.cols(); i++)
    {
        int* id = new int[nk];
        fScalar *dist = new fScalar[nk];
        kdtree.query(X.col(i).data(), nk, id, dist);
        VectorX k_dist = Eigen::Map<VectorX>(dist, nk);
        igl::median(k_dist.tail(nk - 1), X_nearest[i]);
        delete[]id;
        delete[]dist;
    }
    fScalar med;
    igl::median(X_nearest, med);
    return med;
}


void Registration::nonrigid_init()
{
    mat_U0_.resize(3, n_src_vertex_);

    // welsch parameters
    weight_d_.resize(n_src_vertex_);//weight_d_是n维向量
    weight_s_.resize(src_mesh_->n_halfedges());

    // Initialize correspondences
    InitCorrespondence(correspondence_pairs_);
    std::cout<<"变形配准的点对初始化："<<correspondence_pairs_.size()<<std::endl;

    VectorX init_nus(correspondence_pairs_.size());
    for(size_t i = 0; i < correspondence_pairs_.size(); i++)
    {
        Vector3 closet = correspondence_pairs_[i].position;
        init_nus[i] = (src_mesh_->point(src_mesh_->vertex_handle(correspondence_pairs_[i].src_idx))
                    - Vec3(closet[0], closet[1], closet[2])).norm();//变形后的点云和目标点云的距离
    }
    igl::median(init_nus, pars_.Data_nu);//计算特征向量的中间值


    if(pars_.calc_gt_err&&n_src_vertex_ == n_tar_vertex_)
    {
        VectorX gt_err(n_src_vertex_);
        for(int i = 0; i < n_src_vertex_; i++)
        {
            gt_err[i] = (src_mesh_->point(src_mesh_->vertex_handle(i)) - tar_mesh_->point(tar_mesh_->vertex_handle(i))).norm();
        }//计算模型整体的误差
        
        pars_.init_gt_mean_errs = std::sqrt(gt_err.squaredNorm()/n_src_vertex_);
        pars_.init_gt_max_errs = gt_err.maxCoeff();
    }
}



// Rigid Registration
fScalar Registration::DoRigid()
{
    //构建点云矩阵3*n
    Matrix3X rig_tar_v = Matrix3X::Zero(3, n_src_vertex_);
    Matrix3X rig_src_v = Matrix3X::Zero(3, n_src_vertex_);
    Affine3 old_T;//构建Affine矩阵

    //std::cout << "before for loop done !" << std::endl;
	corres_pair_ids_.setZero();
    int nn=0;
    //在允许的最大循环里进行配准
    for(int iter = 0; iter < pars_.rigid_iters; iter++)
    {
        nn++;
        for (size_t i = 0; i < correspondence_pairs_.size(); i++)
        {
            //将source点云和配对的target点云存入矩阵
            rig_src_v.col(i) = Eigen::Map<Vector3>(src_mesh_->point(src_mesh_->vertex_handle(correspondence_pairs_[i].src_idx)).data(), 3, 1);
            rig_tar_v.col(i) = correspondence_pairs_[i].position;
            corres_pair_ids_[correspondence_pairs_[i].src_idx] = 1;//点对的置信度
        }
        // std::cout<<"correspondence_pairs_.size()"<<correspondence_pairs_.size()<<std::endl;
        old_T = rigid_T_;
        //点对点刚体配准
        rigid_T_ = point_to_point(rig_src_v, rig_tar_v, corres_pair_ids_);

        //如果变换矩阵相差很小则停止
        if((old_T.matrix() - rigid_T_.matrix()).norm() < 1e-3)
        {
            std::cout<<"变化太小"<<std::endl;
            break;
        }
        // for (size_t i = 0; i < 4; i++)
        // {
        //     for (size_t j = 0; j < 4; j++)
        //     {
        //         std::cout<<rigid_T_(i,j)<<" ";
        //     }
        //     std::cout<<std::endl;            
        // }

        #pragma omp parallel for
        
        for (int i = 0; i < n_src_vertex_; i++)
        {
            Vec3 p = src_mesh_->point(src_mesh_->vertex_handle(i));//将更新后的点云存入p中
            // Vector3 temp = Eigen::Map<Vector3>(p.data(), 3);//变换后的source点云
            Vector3 temp = rigid_T_ * Eigen::Map<Vector3>(p.data(), 3);//变换后的source点云
            p[0] = temp[0];
            p[1] = temp[1];
            p[2] = temp[2];
            src_mesh_->set_point(src_mesh_->vertex_handle(i), p);//更新src_mesh_的模型
        }
        //更新点对对应关系
        // Find correspondence
        FindClosestPoints(correspondence_pairs_);
        SimplePruning(correspondence_pairs_, pars_.use_distance_reject, pars_.use_normal_reject);
    }
    /*
    OpenMesh::IO::write_mesh(*src_mesh_,"hh.obj");
    std::ofstream file1("pcdsource2.txt");
    std::ofstream file2("pcdtarget2.txt");
    Mesh source_mesh,target_mesh;
    workutils::read_data(name1, source_mesh);
    workutils::read_data(name2, target_mesh);
    Mesh* source_mesh_;
    Mesh* target_mesh_;
    source_mesh_=&source_mesh;
    target_mesh_=&target_mesh;

    std::cout<<"点对数："<<correspondence_pairs_.size()<<std::endl;
    for(auto it=correspondence_pairs_.begin();it!=correspondence_pairs_.end();it++)
    {
        // file2<<it->position(0)<<" "<<it->position(1)<<" "<<it->position(2)<<std::endl;
        file2<<target_mesh_->point(target_mesh_->vertex_handle(it->tar_idx))[0]+0.1<<" "
        <<target_mesh_->point(target_mesh_->vertex_handle(it->tar_idx))[1]+0.1<<" "
        <<target_mesh_->point(target_mesh_->vertex_handle(it->tar_idx))[2]+0.1<<std::endl;
        file1<<source_mesh_->point(source_mesh_->vertex_handle(it->src_idx))[0]<<" "
        <<source_mesh_->point(source_mesh_->vertex_handle(it->src_idx))[1]<<" "
        <<source_mesh_->point(source_mesh_->vertex_handle(it->src_idx))[2]<<std::endl;
    }
    */

    std::cout<<"刚性配准次数："<<nn<<std::endl;
    return 0;
}


//查找最近点并构建点对
void Registration::FindClosestPoints(VPairs & corres)
{
    //将点对的个数定义为源模型的顶点个数
    corres.resize(n_src_vertex_);

    #pragma omp parallel for

    //为源模型的顶点寻找点对
    for (int i = 0; i < n_src_vertex_; i++)
    {
        fScalar mini_dist;
        //通过KDtree寻找距离source mesh点对应的最近的target上的点，并将最小距离存入mini_dist 
        int idx = target_tree->closest(src_mesh_->point(src_mesh_->vertex_handle(i)).data(), mini_dist);
        Closest c;
        c.src_idx = i;
        c.tar_idx= idx;//modified,记录下目标模型的顶点id
        c.position = tar_points_.col(idx);//存入目标模型的顶点位置
        //记录下目标模型上该点的法线
        c.normal = Vec2Eigen(tar_mesh_->normal(tar_mesh_->vertex_handle(idx)));
        //按照距离最小原则寻找到source点云所对应的target的点对，并存入corres.
        corres[i] = c;
    }
    // std::cout<<"查找点对:"<<corres.size()<<std::endl;
}

//简单优化点对
void Registration::SimplePruning(VPairs & corres, bool use_distance = true, bool use_normal = false)
{
    // Distance and normal
    VectorX tar_min_dists(n_tar_vertex_);//tar_min_dists为n_t行的矩阵
    tar_min_dists.setConstant(1e10);//设置矩阵中的所有元素为1e10;
    Eigen::VectorXi min_idxs(n_tar_vertex_);//min_idxs为n_t维的向量
    min_idxs.setConstant(-1);//min_idxs所有值为-1
    src_mesh_->update_vertex_normals();//计算update_face_normals() , update_halfedge_normals() and update_vertex_normals() if the normals exist.

    VectorX corres_idx = VectorX::Zero(n_src_vertex_);//corres_idx为n_s维的零向量
    //
    for(size_t i = 0; i < corres.size(); i++)
    {
        Vector3 closet = corres[i].position;
        //计算点对间的距离
        fScalar dist = (src_mesh_->point(src_mesh_->vertex_handle(corres[i].src_idx))
                       - Eigen2Vec(closet)).norm();
        // std::cout<<dist<<std::endl;

        Vec3 src_normal = src_mesh_->normal(src_mesh_->vertex_handle(corres[i].src_idx));
        Vec3 tar_normal = Eigen2Vec(corres[i].normal);
        //计算两个向量的夹角
        fScalar angle = acos(src_normal | tar_normal / (src_normal.norm()*tar_normal.norm()));

        //如果满足某些条件，相应的点对会通过corres_idx被标记
        if((!use_distance || dist < pars_.distance_threshold)
            && (!use_normal || src_mesh_->n_faces() == 0 || angle < pars_.normal_threshold))
        {
            corres_idx[i] = 1;
            // std::cout<<i<<std::endl;
        }
    }
    //没有使用
    if(pars_.use_fixedvex)
    {
        for(size_t i = 0; i < pars_.fixed_vertices.size(); i++)
        {
            int idx = pars_.fixed_vertices[i];
            corres_idx[idx] = 0;
        }
    }

    //清理不满足条件的点对
    VPairs corres2;
    for (auto it = corres.begin(); it != corres.end(); it++)
    {
        //搜索所有符合条件的点对
        if (corres_idx[(*it).src_idx] == 1)
        {
            corres2.push_back(*it);
        }
    }
    corres.clear();
    corres = corres2;
}


void Registration::LandMarkCorres(VPairs & corres)
{
    corres.clear();
    if (pars_.landmark_src.size() != pars_.landmark_tar.size())
    {
        std::cout << "Error: landmark data wrong!!" << std::endl;
    }
    n_landmark_nodes_ = pars_.landmark_tar.size();
    for (int i = 0; i < n_landmark_nodes_; i++)
    {
        Closest c;
        c.src_idx = pars_.landmark_src[i];
        OpenMesh::VertexHandle vh = tar_mesh_->vertex_handle(pars_.landmark_tar[i]);

        if (c.src_idx > n_src_vertex_ || c.src_idx < 0)
            std::cout << "Error: source index in Landmark is out of range!" << std::endl;
        if (vh.idx() < 0)
            std::cout << "Error: target index in Landmark is out of range!" << std::endl;

        c.position = Vec2Eigen(tar_mesh_->point(vh));
        c.normal = Vec2Eigen(tar_mesh_->normal(vh));
        corres.push_back(c);
	}
    std::cout << " use landmark and landmark is ... " << pars_.landmark_src.size() << std::endl;
}

//初始化对应关系
void Registration::InitCorrespondence(VPairs & corres)
{
    FindClosestPoints(corres);//通过最小距离初始化点对
    std::cout<<"修剪前："<<corres.size()<<std::endl;
    SimplePruning(corres);//优化点对
    std::cout<<"修剪后："<<corres.size()<<std::endl;

    //没有landmark
    if(pars_.use_landmark)
    {
        corres.clear();
        for(int i = 0; i < pars_.landmark_src.size(); i++)
        {
            Closest c;
            c.src_idx = pars_.landmark_src[i];
            c.tar_idx = pars_.landmark_tar[i];
            c.position = tar_points_.col(c.tar_idx);
            c.normal = Vec2Eigen(tar_mesh_->normal(tar_mesh_->vertex_handle(c.tar_idx)));//需要target是一个曲面
            corres.push_back(c);
        }
    }
}

// *type: 0 :median, 1: average
fScalar Registration::CalcEdgelength(Mesh* mesh, int type)
{
    fScalar med;
    if(mesh->n_faces() > 0)
    {
        VectorX edges_length(mesh->n_edges());
        for(size_t i = 0; i < mesh->n_edges();i++)
        {
            OpenMesh::VertexHandle vi = mesh->from_vertex_handle(mesh->halfedge_handle(mesh->edge_handle(i),0));
            OpenMesh::VertexHandle vj = mesh->to_vertex_handle(mesh->halfedge_handle(mesh->edge_handle(i),0));
            edges_length[i] = (mesh->point(vi) - mesh->point(vj)).norm();
        }
        if (type == 0)
            igl::median(edges_length, med);
        else
            med = edges_length.mean();
    }
    else
    {
        // source is mesh, target may be point cloud.
        VectorX edges_length(mesh->n_vertices());
        int nk = 7;
        for(size_t i = 0; i<mesh->n_vertices(); i++)
        {
            int* id = new int[nk];
            fScalar *dist = new fScalar[nk];
            target_tree->query(mesh->point(mesh->vertex_handle(i)).data(), nk, id, dist);
            VectorX k_dist = Eigen::Map<VectorX>(dist, nk);
            if (type == 0)
                igl::median(k_dist.tail(nk - 1), edges_length[i]);
            else
                edges_length[i] = k_dist.tail(nk - 1).mean();
            delete[]id;
            delete[]dist;
        }
        if (type == 0)
            igl::median(edges_length, med);
        else
            med = edges_length.mean();
    }
    return med;
}

/// @param Source (one 3D point per column)
/// @param Target (one 3D point per column)
/// @param Confidence weights
template <typename Derived1, typename Derived2, typename Derived3>
Affine3 Registration::point_to_point(Eigen::MatrixBase<Derived1>& X,
    Eigen::MatrixBase<Derived2>& Y,
    const Eigen::MatrixBase<Derived3>& w) {
    /// Normalize weight vector
    VectorX w_normalized = w / w.sum();
    /// De-mean
    Vector3 X_mean, Y_mean;
    for (int i = 0; i<3; ++i) {
        X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
        Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
    }
    X.colwise() -= X_mean;
    Y.colwise() -= Y_mean;
    // std::cout<<X.row(0)<<std::endl;

    // std::cout<<Y_mean<<std::endl;
    /// Compute transformation
    Affine3 transformation;
    Matrix33 sigma = X * w_normalized.asDiagonal() * Y.transpose();
    Eigen::JacobiSVD<Matrix33> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
    if (svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
        Vector3 S = Vector3::Ones(); S(2) = -1.0;
        transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
    }
    else {
        transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
    }
    transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
    // std::cout<<transformation.translation()<<std::endl;
    /// Re-apply mean
    X.colwise() += X_mean;
    Y.colwise() += Y_mean;
    /// Return transformation
    return transformation;
}
