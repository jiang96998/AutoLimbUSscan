#ifndef REGISTRATION_H_
#define REGISTRATION_H_
#include "tools/nanoflann.h"
#include "tools/tools.h"
#include "tools/Types.h"
#include "tools/OmpHelper.h"

class Registration
{
public:
    Registration();
    virtual ~Registration();

    Mesh* src_mesh_;
    Mesh* tar_mesh_;
    int n_src_vertex_;
    int n_tar_vertex_;
    int n_landmark_nodes_;

    //构建最近点的结构
    struct Closest{
        int src_idx; // vertex index from source model
        int tar_idx; // face index from target model
        Vector3 position;
        Vector3 normal;
    };
    typedef std::vector<Closest> VPairs;
    VPairs correspondence_pairs_;//储存最近点结构的vector
protected:
    // non-rigid Energy function paras
    VectorX weight_d_;	 // E_data weight n * n diag(w1, w2, ... wn)
    VectorX weight_s_;	 // E_reg weight |e| * |e| diag;
    VectorX weight_4o_;	 // E_rot weight

    MatrixXX grad_X_;	 // gradient of E about X; 4n * 3
    Eigen::SparseMatrix<fScalar> mat_A0_;	 // intial Hessian approximation 4r * 4r
    MatrixXX direction_;  // descent direction 4n * 3
    MatrixXX tar_points_; // target_mesh  3*m;
    MatrixXX mat_VU_;         // U'V  4n*3
    MatrixXX mat_U0_;      // initial mat_U_; 3 * n

    KDtree* target_tree; // correspondence paras

    // Rigid paras
    Affine3 rigid_T_;  // rigid registration transform matrix

    // Check correspondence points
    VectorX corres_pair_ids_;//列数为1的矩阵
    // VPairs correspondence_pairs_;//储存最近点结构的vector
    Eigen::SparseMatrix<fScalar> sub_V_;
    MatrixXX sub_U_;
    int current_n_;

    // dynamic welsch parasmeters
    bool init_nu;
    fScalar end_nu;
    fScalar nu;

    bool update_tarGeotree;


public:
    // adjusted paras
    bool use_cholesky_solver_;
    bool use_pardiso_;
    RegParas pars_;

public:
    void nonrigid_init();
    virtual fScalar DoNonRigid() { return 0.0; }
    fScalar DoRigid();
    void rigid_init(Mesh& src_mesh, Mesh& tar_mesh, RegParas& paras);
    virtual void Initialize(){}
    std::string name1,name2;

private:
    Eigen::VectorXi  init_geo_pairs;

protected:
    //point to point rigid registration
    template <typename Derived1, typename Derived2, typename Derived3>
    Affine3 point_to_point(Eigen::MatrixBase<Derived1>& X,
        Eigen::MatrixBase<Derived2>& Y, const Eigen::MatrixBase<Derived3>& w);

    // Find correct correspondences
    void InitCorrespondence(VPairs & corres);
    void FindClosestPoints(VPairs & corres);

    // Pruning method
    void SimplePruning(VPairs & corres, bool use_distance, bool use_normal);

    // Use landmark;
    void LandMarkCorres(VPairs & correspondence_pairs);

    // Aux_tool function
    fScalar CalcEdgelength(Mesh* mesh, int type);
    template<typename Derived1>
    fScalar FindKnearestMed(Eigen::MatrixBase<Derived1>& X, int nk);
};
#endif
