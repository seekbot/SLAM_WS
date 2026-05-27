#include <gtest/gtest.h>
#include <pcl/common/transforms.h>
#include "loop_closure.h"

/* === Unit test with synthetic clouds === */

namespace {
    // Helper - synthesise a "scene" cloud: random points in a 20m x 20m x 5m volume
    PointCloudType makeSyntheticScene(int seed, int n_pts = 5000) {
        std::srand(seed);
        PointCloudType cloud;
        cloud.reserve(n_pts);
        for (int i = 0; i < n_pts; ++i) {
            PointType p;
            p.x = 20.f * ((float)std::rand()/RAND_MAX - 0.5f);
            p.y = 20.f * ((float)std::rand()/RAND_MAX - 0.5f);
            p.z = 5.f * ((float)std::rand()/RAND_MAX - 0.5f);
            cloud.push_back(p);
        }
        return cloud;
    }

    LoopClosureConfig makeTestConfig() {
        LoopClosureConfig c;
        c.voxel_res_ = 0.3;
        c.num_submap_keyframes_ = 3;
        c.enable_submap_matching_ = true;
        c.enable_quatro_ = false;
        c.gicp_config_.icp_score_thr_ = 0.3;
        return c;
    }
} // namespace

/* === Test 1: Identical clouds -> loop accepted === */
TEST(LoopClosureUnit, IdenticalCloudsRecoverIdentity) {
    LoopClosure lc(makeTestConfig());
    PointCloudType scene = makeSyntheticScene(42);

    // Build two PosePcd "keyframes" - same cloud, same pose. ICP must converge.
    PosePcd kf0; kf0.idx_ = 0; kf0.pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    kf0.pcd_ = scene;
    PosePcd kf1 = kf0; kf1.idx_ = 1;

    // Bypass fetchCandidateKeyframeIdx - we KNOW the candidate is kf0
    std::vector<PosePcd> kfs{kf0, kf1};
    auto out = lc.performLoopClosure(kf1, kfs, 0);
    EXPECT_TRUE(out.is_valid_);
    EXPECT_TRUE(out.is_converged_);
    EXPECT_LT(out.score_, 0.1);     // near-zero fitness

    // Recovered transform ≈ identity
    Eigen::Matrix4d err = out.pose_btwn_eig_ - Eigen::Matrix4d::Identity();
    EXPECT_LT(err.norm(), 1e-2);
}

/* === Test 2: Known transform recovered to within tolerance === */
TEST(LoopClosureUnit, KnownTransformRecovered) {
    LoopClosure lc(makeTestConfig());
    PointCloudType scene = makeSyntheticScene(7);

    PosePcd kf0; kf0.idx_ = 0;
    kf0.pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    kf0.pcd_ = scene;

    /** Apply a known "drift" (e.g., 0.5 m offset, 10 deg yaw) to kf1's known pose only.
     *  NOTE: pcd_ stays in body frame(=scene). setSrcAndDstCloud() will project
     *  each pcd_ into world via its pose_corrected_eig_, so the test must NOT
     *  pre-transform the cloud (that would apply T twice).
     * */ 
    Eigen::Affine3d T = Eigen::Affine3d::Identity();
    T.translate(Eigen::Vector3d(0.5, 0, 0));
    T.rotate(Eigen::AngleAxisd(10 * M_PI/180, Eigen::Vector3d::UnitZ()));
    PosePcd kf1; kf1.idx_ = 1;
    kf1.pose_corrected_eig_ = T.matrix();   // the "drifted" estimate
    kf1.pcd_ = scene;                       // SAME BODY-frame scene (no double transform)                                

    std::vector<PosePcd> kfs{kf0, kf1};
    auto out = lc.performLoopClosure(kf1, kfs, 0);
    ASSERT_TRUE(out.is_valid_);
    /**
     * Both clouds are the same scene; only the world poses differ by T.
     * world(src) = T · scene, world(dst) = scene, so GICP's pose_between ≈ T^-1.
     */
    Eigen::Matrix4d expect = T.inverse().matrix();
    EXPECT_LT((out.pose_btwn_eig_ - expect).norm(), 0.1);
}

/* === Test 3: Unrelated clouds -> loop REJECTED (the gate's negative case) === */
TEST(LoopClosureUnit, UnrelatedCloudsRejected) {
    LoopClosure lc(makeTestConfig());
    PosePcd a; a.idx_ = 0; a.pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    a.pcd_ = makeSyntheticScene(1);

    /** Make b's scene CLEARLY different - different seed AND spatially shifted
     * by 50m so the two clouds can't accidentally overlap after voxelisation.
     */
    PosePcd b; b.idx_ = 1; b.pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d shift = Eigen::Matrix4d::Identity();
    shift(0,3) = 50.0;
    b.pose_corrected_eig_ = shift;
    b.pcd_ = makeSyntheticScene(9999);

    std::vector<PosePcd> kfs{a, b};
    auto out = lc.performLoopClosure(b, kfs, 0);
    EXPECT_FALSE(out.is_valid_);    // THE gate must catch this
}

/* === Test 4: Voxelisation actually shrinks the cloud (regression) === */
TEST(LoopClosureUnit, VoxelisationReducesCloudSize) {
    auto cfg = makeTestConfig();
    cfg.voxel_res_ = 1.0;           // big voxel -> aggressive downsample
    LoopClosure lc(cfg);
    PointCloudType scene = makeSyntheticScene(3, 20000);
    PosePcd kf; kf.pcd_ = scene; kf.pose_corrected_eig_ = Eigen::Matrix4d::Identity();
    auto [src, dst] = lc._test_setSrcAndDstCloud(
        {kf, kf}, 0, 1, 0, cfg.voxel_res_, false);
    EXPECT_LT((int)src.size(), (int)scene.size() / 2);
    EXPECT_LT((int)dst.size(), (int)scene.size() / 2);
}