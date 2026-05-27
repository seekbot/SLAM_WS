#pragma once
#include <limits>
#include <mutex>
#include <tuple>
#include "pose_pcd.hpp"
#include "Scancontext.h"
#include <nano_gicp/nano_gicp.hpp>

struct GicpConfig {
    int nano_thread_number_ = 4;
    int nano_correspondences_number_ = 20;
    int nano_max_iter_ = 32;
    double max_corr_dist_ = 2.0;
    double transformation_epsilon_ = 0.01;
    double euclidean_fitness_epsilon_ = 0.01;
    double icp_score_thr_ = 0.3;    // THE accept threshold
};

struct LoopClosureConfig {
    double voxel_res_ = 0.3;
    int num_submap_keyframes_ = 10;
    double scancontext_max_correspondence_dist_ = 15.0;
    bool enable_submap_matching_ = true;
    bool enable_quatro_ = false;
    GicpConfig gicp_config_;
};

struct RegistrationOutput {
    bool is_valid_ = false;
    bool is_converged_ = false;
    double score_ = std::numeric_limits<double>::infinity();
    Eigen::Matrix4d pose_btwn_eig_ = Eigen::Matrix4d::Identity();
};

class LoopClosure {
public:
    explicit LoopClosure(const LoopClosureConfig& cfg);
    void updateScancontext(const PointCloudType& cloud);
    int fetchCandidateKeyframeIdx(const PosePcd& query, const std::vector<PosePcd>& keyframes);
    RegistrationOutput performLoopClosure(const PosePcd& query, const std::vector<PosePcd>& keyframes, int cand_idx);

    /* === Testing-only access === */
    // Forwards to private helper; same signature. Used by loop_closure_test.cpp
    std::pair<PointCloudType, PointCloudType> _test_setSrcAndDstCloud(
        const std::vector<PosePcd>& keyframes, 
        int src_idx, int dst_idx,
        int submap_range, double voxel_res,
        bool use_submaps) 
    {
        return setSrcAndDstCloud(keyframes, src_idx, dst_idx, submap_range, voxel_res, use_submaps);
    }

private:
    // internal helpers used by performLoopClosure (no node outside the class should call them directly)
    std::pair<PointCloudType, PointCloudType> setSrcAndDstCloud(
        const std::vector<PosePcd>& keyframes,
        int src_idx, int dst_idx,
        int submap_range, double voxel_res,
        bool use_submaps);
    
    RegistrationOutput icpAlignment(const PointCloudType& src, const PointCloudType& dst);
    RegistrationOutput coarseToFineAlignment(const PointCloudType& src, const PointCloudType& dst);

    LoopClosureConfig config_;
    SCManager sc_manager_;
    nano_gicp::NanoGICP<PointType, PointType> nano_gicp_;
    PointCloudType aligned_;
};