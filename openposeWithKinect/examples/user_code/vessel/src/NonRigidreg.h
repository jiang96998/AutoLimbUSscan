#ifndef QN_WELSCH_H_
#define QN_WELSCH_H_
#include "Registration.h"
#include "tools/nodeSampler.h"

typedef Eigen::SparseMatrix<fScalar> SparseMatrix;

class NonRigidreg : public Registration
{
public:
    NonRigidreg();
    ~NonRigidreg();
    virtual fScalar DoNonRigid();
    virtual void Initialize();

private:
    fScalar welsch_error(fScalar nu1, fScalar nu2);
    fScalar welsch_energy(VectorX& r, fScalar p);
    void welsch_weight(VectorX& r, fScalar p);

    void LBFGS(int iter, MatrixXX & dir) const;
    int QNSolver(fScalar& data_err, fScalar& smooth_err, fScalar& orth_err);

    fScalar sample_energy(fScalar& data_err, fScalar& smooth_err, fScalar& orth_err);
	void   sample_gradient();
    void   update_R();
    fScalar SetMeshPoints(Mesh* mesh, const MatrixXX & target, MatrixXX& cur_v);

    Eigen::SimplicialCholesky<SparseMatrix>* ldlt_;

private:
    // BFGS paras
    MatrixXX all_s_; // si = X_i+1 - X_i; all_s_ = 4n * 3lbfgs_m_
    MatrixXX all_t_; // ti = Grad(X_i+1) - Grad(X_i); all_t_ = 4n * 3lbfgs_m_
    int iter_;
    int col_idx_;

	// Sample paras
    int				num_sample_nodes; // (r) the number of sample nodes
	// simplily nodes storage structure
    svr::nodeSampler src_sample_nodes;
	// variable
    MatrixXX		Smat_X_;	// (4r * 3) transformations of sample nodes
    // data term
    SparseMatrix	Weight_PV_; // (n * 4r) the weighting matrix of the sampling points affecting all vertices
	MatrixXX		Smat_P_;    // (n * 3) all sample nodes' coordinates
    // smooth term
    SparseMatrix	Smat_B_;	// (2|e| * 4r) the smooth between nodes, |e| is the edges' number of sample node graph;
    MatrixXX		Smat_D_;	// (2|e| * 3) the different coordinate between xi and xj
    VectorX			Sweight_s_; // (2|e|) the smooth weight;
    // orth term
    MatrixXX		Smat_R_;	// (3r * 3)
    SparseMatrix	Smat_L_;	// (4r * 4r)
    SparseMatrix	Smat_J_;	// (4r * 3r)
    MatrixXX		Smat_UP_;   // aux matrix
    // landmark term ||PV * X - UP||_2^2
    SparseMatrix    Sub_PV_;    // (k * 4r)
    MatrixXX        Sub_UP_;    // (k * 3)

    fScalar          ori_alpha;
    fScalar          ori_beta;
};
#endif
