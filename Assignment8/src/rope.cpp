#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass,
           float k, vector<int> pinned_nodes) {
  // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and
  // containing `num_nodes` nodes.
  Vector2D rope_vec_step = (end - start) / (num_nodes - 1);
  for (int i = 0; i < num_nodes; i++) {
    Vector2D pos = start + rope_vec_step * i;
    this->masses.push_back(new Mass(pos, node_mass, false));
    if (i > 0) {
      this->springs.push_back(new Spring(masses[i - 1], masses[i], k));
    }
  }

  for (auto &i : pinned_nodes) {
    masses[i]->pinned = true;
  }
}

void Rope::simulateEuler(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 2): Use Hooke's law to calculate the force on a node
    Vector2D pos_a = s->m1->position;
    Vector2D pos_b = s->m2->position;
    float ab_length = (pos_b - pos_a).norm();
    Vector2D x = (pos_b - pos_a) / ab_length * (ab_length - s->rest_length);
    Vector2D hook_force = s->k * x;

    s->m1->forces += hook_force;
    s->m2->forces -= hook_force;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      // TODO (Part 2): Add the force due to gravity, then compute the new
      // velocity and position
      float dump_factor = 0.005;
      m->forces += m->mass * gravity;
      m->forces += -dump_factor * m->velocity; // dump
      Vector2D acceleration = m->forces / m->mass;
      auto temp_velocity = m->velocity;      // v(t)
      m->velocity += acceleration * delta_t; // v(t+1)

      // semi-implicit x(t+1) = x(t) + v(t) * dt
      m->position += m->velocity * delta_t;

      // explicit x(t+1) = x(t) + v(t+1) * dt
      // m->position += temp_velocity * delta_t;
    }

    // Reset all forces on each mass
    m->forces = Vector2D(0, 0);
  }
}

void Rope::simulateVerlet(float delta_t, Vector2D gravity) {
  for (auto &s : springs) {
    // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet
    // （solving constraints)
    auto pos_a = s->m1->position;
    auto pos_b = s->m2->position;
    auto unit_ab = (pos_b - pos_a).norm();
    auto x = (pos_b - pos_a) / unit_ab * (unit_ab - s->rest_length);
    auto hook_force = s->k * x;

    s->m1->forces += hook_force;
    s->m2->forces += -hook_force;
  }

  for (auto &m : masses) {
    if (!m->pinned) {
      Vector2D temp_position = m->position;
      // TODO (Part 3.1): Set the new position of the rope mass
      m->forces += m->mass * gravity;
      auto acceleration = m->forces / m->mass;
      m->position = m->position +
                    (1 - 0.00005) * (m->position - m->last_position) +
                    acceleration * delta_t * delta_t;

      m->last_position = temp_position;
    }
    m->forces = Vector2D(0, 0);
  }
}
} // namespace CGL
