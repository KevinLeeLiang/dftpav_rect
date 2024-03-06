//
// Created by garen_lee on 2024/1/29.
/**
 ******************************************************************************
 * @file           : reeds_shepp_path.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/29
 ******************************************************************************
 */
//

#ifndef REEDS_SHEPP_PATH_H
#define REEDS_SHEPP_PATH_H

#pragma once

#include "sbp_node.h"
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

PLANNING_NAMESPACE_START

struct ReedSheppPath {
    DEFINE_SHARDED_PTR(ReedSheppPath)

    std::vector<double> segs_lengths;
    std::vector<char> segs_types;
    double total_length = 0.0;
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> phi;
    std::vector<double> kappa;
    // true for driving forward and false for driving backward
    std::vector<char> gear;

    void reset() {
        total_length = 0.0;
        segs_lengths.clear();
        segs_types.clear();
        x.clear();
        y.clear();
        phi.clear();
        gear.clear();
        kappa.clear();
    }

    void compute_trajectory_cost() {}
};

struct RSPParam {
    bool flag = false;
    double t = 0.0;
    double u = 0.0;
    double v = 0.0;
};

class ReedShepp {
  public:
    DEFINE_SHARDED_PTR(ReedShepp)

    struct Param {
        explicit Param(bool _SCS = true, bool _CSC = true, bool _CCC = true, bool _CCCC = true, bool _CCSC = true,
                       bool _CCSCC = true)
            : SCS(_SCS)
            , CSC(_CSC)
            , CCC(_CCC)
            , CCCC(_CCCC)
            , CCSC(_CCSC)
            , CCSCC(_CCSCC) {}

        bool SCS;
        bool CSC;
        bool CCC;
        bool CCCC;
        bool CCSC;
        bool CCSCC;
    };

    ReedShepp();

    ReedShepp(double max_kappa, double step_size);

    virtual ~ReedShepp() = default;

    bool ShortestRSP(const SearchNode::Ptr &start_node, const SearchNode::Ptr &end_node,
                     const std::shared_ptr<ReedSheppPath> &optimal_path, Param param = Param());
    bool AllRSPs(const SearchNode::Ptr &start_node, const SearchNode::Ptr &end_node,
                 std::vector<ReedSheppPath> *all_possible_path, Param param = Param());

  protected:
    // interpolate them
    bool GenerateRSPs(const SearchNode::Ptr &start_node, const SearchNode::Ptr &end_node, Param param);
    // Set the general profile of the movement primitives
    bool GenerateRSP(const SearchNode::Ptr &start_node, const SearchNode::Ptr &end_node, Param param);
    // Set the general profile of the movement primitives, parallel implementation
    bool GenerateRSPPar(const SearchNode::Ptr &start_node, const SearchNode::Ptr &end_node);
    // Set local exact configurations profile of each movement primitive
    bool GenerateLocalConfigurations(const SearchNode::Ptr &start_node, const SearchNode::Ptr &end_node,
                                     ReedSheppPath *shortest_path) const;

    // Interpolation usde in GenetateLocalConfiguration
    double Interpolation(const int index, const double pd, const char m, const double ox, const double oy,
                         const double ophi, std::vector<double> *px, std::vector<double> *py, std::vector<double> *pphi,
                         std::vector<char> *pgear) const;
    // motion primitives combination setup function
    bool SetRSP(const int size, const double *lengths, const char *types);
    // setRSP parallel version
    bool SetRSPPar(const int size, const double *lengths, const std::string &types, const int idx);
    // Six different combination of motion primitive in Reed Shepp path used in
    // GenerateRSP()
    bool SCS(const double x, const double y, const double phi);
    bool CSC(const double x, const double y, const double phi);
    bool CCC(const double x, const double y, const double phi);
    bool CCCC(const double x, const double y, const double phi);
    bool CCSC(const double x, const double y, const double phi);
    bool CCSCC(const double x, const double y, const double phi);
    // different options for different combination of motion primitivesf
    static void LSL(const double x, const double y, const double phi, RSPParam *param);
    static void LSR(const double x, const double y, const double phi, RSPParam *param);
    static void LRL(const double x, const double y, const double phi, RSPParam *param);
    static void SLS(const double x, const double y, const double phi, RSPParam *param);
    static void LRLRn(const double x, const double y, const double phi, RSPParam *param);
    static void LRLRp(const double x, const double y, const double phi, RSPParam *param);
    void LRSR(const double x, const double y, const double phi, RSPParam *param);
    void LRSL(const double x, const double y, const double phi, RSPParam *param);
    static void LRSLR(const double x, const double y, const double phi, RSPParam *param);
    static std::pair<double, double> calc_tau_omega(const double u, const double v, const double xi, const double eta,
                                                    const double phi);

  protected:
    std::vector<ReedSheppPath> all_possible_paths_;
    int paths_max_size_ = 48;
    size_t paths_size_ = 0;
    double step_size_;
    double max_kappa_;
};

PLANNING_NAMESPACE_END

#endif // NODE_ERROR_REEDS_SHEPP_PATH_H
