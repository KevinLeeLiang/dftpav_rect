//
// Created by garen_lee on 2024/1/25.
/**
 ******************************************************************************
 * @file           : singleton_lookup.h
 * @author         : garen_lee
 * @brief          : None
 * @attention      : None
 * @date           : 2024/1/25
 ******************************************************************************
 */
//

#ifndef SINGLETON_LOOKUP_H
#define SINGLETON_LOOKUP_H
#include "constant.h"
#include <iostream>
#include <math.h>
#include <vector>
#include "../../common.h"


class SingletonLookup {
  public:
    void init();

    inline bool getInitStatus() const { return is_init; }

    static SingletonLookup *GetInstance();

    ~SingletonLookup() { delete m_plan_config; }

  private:
    SingletonLookup(){};
    static SingletonLookup *m_plan_config;
    static common::VehicleParam *ptr_veh_mode_;

  private:
    bool is_init = false;


  public:
    Constants::config collisionLookuptable[Constants::headings * Constants::positions];
};

#endif // NODE_ERROR_SINGLETON_LOOKUP_H
