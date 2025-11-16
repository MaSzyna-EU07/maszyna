#include "rt_model.h"

#include <bvh/v2/bvh.h>
#include <bvh/v2/default_builder.h>
#include <bvh/v2/executor.h>
#include <bvh/v2/stack.h>
#include <bvh/v2/thread_pool.h>
#include <bvh/v2/tri.h>

#include <vector>

#include "Model3d.h"

namespace Rt {

using Bbox = bvh::v2::BBox<float, 3>;
using Vec = bvh::v2::Vec<float, 3>;
using Tri = bvh::v2::PrecomputedTri<float>;
using Node = bvh::v2::Node<float, 3>;

struct RtSubmodel {
  TSubModel const* submodel;
  bvh::v2::Bvh<Node> bvh;
  std::vector<Tri> tris;
};

struct RtModel : IRtModel {
  std::vector<RtSubmodel> submodels;
  std::unordered_map<size_t, int> submodel_mapping;

  RtModel& FromModel3d(TModel3d const* src, NvRenderer const* owner);
  void FromSubmodel(TSubModel const* submodel, NvRenderer const* owner);
  const TSubModel* Intersect(NvRenderer::Renderable const& renderable,
                             glm::dvec3 const& ro,
                             glm::dvec3 const& rd) const override;
};
}  // namespace Rt

std::shared_ptr<Rt::IRtModel> Rt::CreateRtModel(TModel3d const* src,
                                                NvRenderer const* owner) {
  auto model = std::make_shared<RtModel>();
  model->FromModel3d(src, owner);
  return model;
}

Rt::RtModel& Rt::RtModel::FromModel3d(TModel3d const* src,
                                      NvRenderer const* owner) {
  FromSubmodel(src->Root, owner);
  return *this;
}

void Rt::RtModel::FromSubmodel(TSubModel const* submodel,
                               NvRenderer const* owner) {
  if (!submodel) {
    return;
  }
  for (; !!submodel; submodel = submodel->Next) {
    if (submodel->fSquareMinDist >
        0.) {  // Only LOD0 contributes to RT structure
      continue;
    }
    if (submodel->eType == GL_TRIANGLES ||
        submodel->eType == GL_TRIANGLE_STRIP) {
      submodel_mapping[submodel->m_geometry.handle] = submodels.size();
      auto& dest = submodels.emplace_back();
      auto const& verts = owner->Vertices(submodel->m_geometry.handle);
      auto const& indices = owner->Indices(submodel->m_geometry.handle);
      auto add_triangle = [&](glm::vec3 const& a, glm::vec3 const& b,
                              glm::vec3 const& c) {
        dest.tris.emplace_back(Vec(a.x, a.y, a.z), Vec(b.x, b.y, b.z),
                               Vec(c.x, c.y, c.z));
      };
      bool const is_indexed = !indices.empty();
      for (int idx = 0, num = is_indexed ? indices.size() : verts.size();
           idx < num; idx += 3) {
        std::array<int, 3> tri_indices;
        for (int i = 0; i < 3; ++i) {
          if (is_indexed)
            tri_indices[i] = indices[idx + i];
          else
            tri_indices[i] = idx + i;
        }
        add_triangle(verts[tri_indices[0]].position,
                     verts[tri_indices[1]].position,
                     verts[tri_indices[2]].position);
      }

      bvh::v2::ThreadPool thread_pool;
      bvh::v2::ParallelExecutor executor(thread_pool);

      // Get triangle centers and bounding boxes (required for BVH builder)
      std::vector<Bbox> bboxes(dest.tris.size());
      std::vector<Vec> centers(dest.tris.size());
      executor.for_each(0, dest.tris.size(), [&](size_t begin, size_t end) {
        for (size_t i = begin; i < end; ++i) {
          bboxes[i] = dest.tris[i].get_bbox();
          centers[i] = dest.tris[i].get_center();
        }
      });

      dest.submodel = submodel;

      bvh::v2::DefaultBuilder<Node>::Config config;
      config.quality = bvh::v2::DefaultBuilder<Node>::Quality::High;
      dest.bvh = bvh::v2::DefaultBuilder<Node>::build(thread_pool, bboxes,
                                                      centers, config);
    }
    FromSubmodel(submodel->Child, owner);
  }
}

const TSubModel* Rt::RtModel::Intersect(
    NvRenderer::Renderable const& renderable, glm::dvec3 const& ro,
    glm::dvec3 const& rd) const {
  TSubModel const* result = nullptr;
  bvh::v2::SmallStack<bvh::v2::Bvh<Node>::Index, 64> stack;
  bvh::v2::Ray<float, 3> ray{};
  ray.tmax = std::numeric_limits<float>::max();
  for (auto const& item : renderable.m_items) {
    if (auto const it = submodel_mapping.find(item.m_geometry);
        it != submodel_mapping.end()) {
      auto const& submodel = submodels[it->second];
      glm::dmat4 const submodel_matrix =
          glm::inverse(glm::dmat4(item.m_transform));
      glm::vec3 ro_local = submodel_matrix * glm::dvec4(ro, 1.);
      glm::vec3 rd_local = normalize(submodel_matrix * glm::dvec4(rd, 0.));
      ray.org = {ro_local.x, ro_local.y, ro_local.z};
      ray.dir = {rd_local.x, rd_local.y, rd_local.z};
      float u, v;
      size_t prim_id = std::numeric_limits<size_t>::max();
      stack.size = 0;
      submodel.bvh.intersect<true, false>(
          ray, submodel.bvh.get_root().index, stack,
          [&](size_t const begin, size_t const end) {
            for (size_t i = begin; i < end; ++i) {
              size_t const j = submodel.bvh.prim_ids[i];
              if (auto hit = submodel.tris[j].intersect(ray)) {
                prim_id = i;
                std::tie(ray.tmax, u, v) = *hit;
                result = submodel.submodel;
              }
            }
            return prim_id != std::numeric_limits<size_t>::max();
          });
    }
  }
  return result;
}