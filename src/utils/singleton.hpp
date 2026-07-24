/**
 * @file singleton.hpp
 * @author Pumpkin Rice (pumpkin_rice@163.com)
 * @brief 单例模式封装
 * @version 0.1
 * @date 2021-10-25
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef SIGNLETION_H_
#define SIGNLETION_H_

namespace pica {
    namespace {
        /**
         * @brief 获取一个实例
         * 
         * @tparam T 实例类型
         * @tparam X 
         * @tparam N 
         * @return T& 
         */
        template<class T, class X, int N>
        T& GetInstanceX() {
            static T v;
            return v;
        }

#ifdef _GLIBCXX_MEMORY
        /**
         * @brief 获取一个动态分配的实例
         * 
         * @tparam T 实例类型
         * @tparam X 
         * @tparam N 
         * @return std::shared_ptr<T> 
         */
        template<class T, class X, int N>
        std::shared_ptr<T> GetInstancePtr() {
            static std::shared_ptr<T> v(new T);
            return v;   
        }
#endif
    }

    /**
     * @brief 单例模式封装类
     * 
     * @tparam T 
     * @tparam X 
     * @tparam N 
     */
    template<class T, class X = void, int N = 0>
    class Singleton {
    public:
        /**
         * @brief 返回单例裸指针
         * 
         * @return T* 
         */
        static T* GetInstance() {
            static T v;
            return &v;
        }
    };

#ifdef _GLIBCXX_MEMORY

    template<class T, class X = void, int N = 0>
    class SingletonPtr {
    public:
        static std::shared_ptr<T> GetInstance() {
            static std::shared_ptr<T> v(new T);
            return v;
            // return GetInstancePtr<T, X, N>();
        }
    };

#endif
};

#endif /* SYLAR_SIGNLETION_H_ */
