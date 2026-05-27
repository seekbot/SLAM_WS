#include "loop_closure.h"

void LoopClosure::updateScancontext(const PointCloudType& cloud) {
    sc_manager_.makeAndSaveScancontextAndKeys(cloud);
}

int LoopClosure::fetchCandidateKeyframeIdx(
        const PosePcd& query, const std::vector<PosePcd>& keyframes) 
{
    /**Two gates deliberately
     * ScanContext alone occasionally matches structurally-similar but 
     * distant places (e.g., two similar corridors). The distance (2nd) 
     * gate uses the current best estiamte of both poses to throw those 
     * out cheaply before the expensive registration.  
     */
    auto [idx, yaw] = sc_manager_.detectLoopClosureIDGivenScan(query.pcd_);
    (void)yaw;  // yaw is informational; GICP refines anyway
    if (idx < 0) return -1;     // Early return if SC found nothing

    // 2nd gate: SC can match structurally similar but distant places.
    // Reject if the candidate's CORRECTED position is too far from ours.
    Eigen::Vector3d dist = 
        keyframes[idx].pose_corrected_eig_.block<3,1>(0,3) - 
        query.pose_corrected_eig_.block<3,1>(0,3);
    
    if (dist.norm() < config_.scancontext_max_correspondence_dist_) return idx;
    
    return -1;
}

// Scan-to-submap: Each stored cloud is re-projected with its CORRECTED pose at the moment of matching
// Voxelizing both sides keeps GICP fast and desnity-balanced.
std::pair<PointCloudType, PointCloudType>LoopClosure::setSrcAndDstCloud(
    const std::vector<PosePcd>& keyframes,
    int src_idx, int dst_idx,
    int submap_range, double voxel_res,
    bool use_submap)
{
    // src = query keyframe alone (re-projected to world via corrected pose)
    PointCloudType src_accum = 
        transformPcd(keyframes[src_idx].pcd_, keyframes[src_idx].pose_corrected_eig_);

    // dst = candidate +/- submap_range neighbors, accumulated in WORLD frame
    PointCloudType dst_accum;
    if (!use_submap) {
        dst_accum = 
            transformPcd(keyframes[dst_idx].pcd_, keyframes[dst_idx].pose_corrected_eig_);
    } else {
        for (int i = dst_idx - submap_range; i <= dst_idx + submap_range; ++i) {
            if (i >= 0 && (int)keyframes.size() - 1)
                dst_accum += transformPcd(keyframes[i].pcd_, keyframes[i].pose_corrected_eig_);
        }
    }
    return {
        *voxelizePcd(src_accum, voxel_res),
        *voxelizePcd(dst_accum, voxel_res)
    };
}

// Verifier
RegistrationOutput LoopClosure::icpAlignment(
    const PointCloudType& src, const PointCloudType& dst
) 
{
    /** ScanContext's SC_DIST_THRES is a filter, not a decision.
     * Decision: A loop is real only if GICP converges AND the fitness score clears icp_score_thr_.
     * One false loop that bypasses this gate enters iSAM2 and folds the entire trajectory irreversibly.
     * When building this, deliberately feed it two unrelated clouds & confirm is_valid_ == false before
     * wiring anything to the graph. 
     */
    RegistrationOutput out;
    auto src_p = std::make_shared<PointCloudType>(src);
    auto dst_p = std::make_shared<PointCloudType>(dst);

    nano_gicp_.setInputSource(src_p);
    nano_gicp_.calculateSourceCovariances();
    nano_gicp_.setInputTarget(dst_p);
    nano_gicp_.calculateTargetCovariances();
    nano_gicp_.align(aligned_);

    out.score_ = nano_gicp_.getFitnessScore();

    // THE gate: converged AND fitness below threshold (lower = better)
    if (nano_gicp_.hasConverged() && 
        out.score_ < config_.gicp_config_.icp_score_thr_) 
    {
        out.is_valid_ = true;
        out.is_converged_ = true;
        out.pose_btwn_eig_ = nano_gicp_.getFinalTransformation().cast<double>();
    }
    return out;     // is_valid_ stays false on rejection 

}

// Glue
RegistrationOutput LoopClosure::performLoopClosure(
    const PosePcd& query, 
    const std::vector<PosePcd>& keyframes,
    int cand_idx) 
{
    RegistrationOutput out;
    if (cand_idx < 0) return out;   // is_valid_ stays false

    auto [src, dst] = setSrcAndDstCloud(
        keyframes, query.idx_, cand_idx,
        config_.num_submap_keyframes_, config_.voxel_res_,
        config_.enable_submap_matching_);
    
    // Quatro++: Segmentation based global registration framework for coarse alignment
    if (config_.enable_quatro_)
        return coarseToFineAlignment(src, dst); // Quatro -> GICP
    
    return icpAlignment(src, dst);  // GICP only
}

// Quatro stub
RegistrationOutput LoopClosure::coarseToFineAlignment(
    const PointCloudType& src, const PointCloudType& dst)
{
    return icpAlignment(src, dst);  // fall through to GICP for now
}