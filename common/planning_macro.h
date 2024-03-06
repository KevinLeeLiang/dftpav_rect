#pragma once

#define PLANNING_NAMESPACE_START                                                                                       \
    namespace haomo {                                                                                                  \
    namespace parking {                                                                                                \
    namespace planning {

#define PLANNING_NAMESPACE_END                                                                                         \
    }                                                                                                                  \
    }                                                                                                                  \
    }

PLANNING_NAMESPACE_START

#define DEFINE_SHARDED_PTR(CLASS_NAME)                                                                                 \
    typedef std::shared_ptr<CLASS_NAME> Ptr;                                                                           \
    typedef std::shared_ptr<const CLASS_NAME> ConstPtr;

#define DEFINE_SINGLETON(CLASSNAME)                                                                                    \
    static CLASSNAME *instance() {                                                                                     \
        static CLASSNAME instance;                                                                                     \
        return &instance;                                                                                              \
    }

PLANNING_NAMESPACE_END
