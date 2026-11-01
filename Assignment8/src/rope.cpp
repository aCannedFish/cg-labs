#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        // 在 start 与 end 之间等间距地生成 num_nodes 个质点
        for (int i = 0; i < num_nodes; ++i)
        {
            double ratio = (num_nodes > 1) ? static_cast<double>(i) / (num_nodes - 1) : 0.0;
            Vector2D position = start + (end - start) * ratio;
            masses.push_back(new Mass(position, node_mass, false));
        }

        // 相邻质点之间用弹簧连接，rest_length 由 Spring 构造函数按初始位置计算
        for (int i = 0; i < num_nodes - 1; ++i)
        {
            springs.push_back(new Spring(masses[i], masses[i + 1], k));
        }

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        // 把 pinned_nodes 中列出的质点标记为固定，它们不参与积分更新
        for (auto &i : pinned_nodes)
        {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        const double damping = 0.00005;

        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            // 胡克定律：两个质点之间的弹簧力与它们当前长度和原长之差成正比，
            // 方向沿两质点的连线，计算出一条边上的力后对两端质点分别累加。
            Vector2D diff = s->m2->position - s->m1->position;
            double length = diff.norm();
            if (length < 1e-8)
            {
                continue;
            }
            // 弹簧被拉长时产生收缩力，力的方向沿两质点连线指向对方
            Vector2D force = diff.unit() * (s->k * (length - s->rest_length));
            s->m1->forces += force;
            s->m2->forces -= force;
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                // 半隐式欧拉：先用受力更新速度，再用新速度更新位置
                m->forces += gravity * m->mass;
                Vector2D acceleration = m->forces / m->mass;
                m->velocity += acceleration * delta_t;

                // TODO (Part 2): Add global damping
                // 全局阻尼让动能逐渐衰减，避免绳子一直抖动
                m->velocity *= (1.0 - damping);

                m->position += m->velocity * delta_t;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        const double damping = 0.00005;

        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            // 将弹簧视为无穷硬，直接把两个质点沿连线方向各移动一半的形变，
            // 使弹簧恢复原长。固定质点不参与移动。
            Vector2D diff = s->m2->position - s->m1->position;
            double length = diff.norm();
            if (length < 1e-8)
            {
                continue;
            }
            Vector2D correction = diff.unit() * ((length - s->rest_length) / 2.0);

            if (s->m1->pinned && !s->m2->pinned)
            {
                s->m2->position -= correction * 2.0;
            }
            else if (!s->m1->pinned && s->m2->pinned)
            {
                s->m1->position += correction * 2.0;
            }
            else if (!s->m1->pinned && !s->m2->pinned)
            {
                s->m1->position += correction;
                s->m2->position -= correction;
            }
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
                // 显式 Verlet 更新，并用阻尼系数削弱上一帧的速度项
                m->position += (m->position - m->last_position) * (1.0 - damping) +
                               gravity * delta_t * delta_t;

                m->last_position = temp_position;
            }
        }
    }
}
