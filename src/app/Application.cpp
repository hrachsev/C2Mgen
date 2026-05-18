#include "Application.h"

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#if defined(__APPLE__)
#include <OpenGL/gl.h>
#elif defined(_WIN32)
#include <GL/gl.h>
#else
#include <GL/gl.h>
#endif

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl2.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

#include <glm/glm.hpp>

#include "core/GeometryTypes.h"
#include "core/JobProgress.h"
#include "core/KdTree.h"
#include "core/Normals.h"
#include "io/ObjWriter.h"
#include "io/PlyLoader.h"
#include "recon/AutoSelector.h"
#include "recon/BallPivoting.h"
#include "recon/CloudStats.h"
#include "recon/GreedyProjection.h"
#include "recon/MarchingCubesRecon.h"
#include "recon/VoxelRemesh.h"
#include "recon/PoissonRecon.h"
#include "render/Viewport3D.h"
#include "system/NativeFileDialogs.h"
#include "ui/OrbitCamera.h"

namespace {

std::mutex g_mu;
PointCloud g_cloud{};
TriangleMesh g_mesh{};

bool g_cloud_normals_from_file = false;

std::atomic<bool> g_busy{false};
std::atomic<float> g_progress{0.f};
ImFont* g_title_font = nullptr;

constexpr const char* kAppName = "C2Mgen";
std::string g_status{"Load a point cloud, then generate a mesh."};
CloudStats g_stats{};
recon::AutoSuggestion g_auto{};

double g_scroll = 0.0;

int g_algo = 0;

BPAParams g_bpa{};
PoissonParams g_poisson{};
MarchingCubesParams g_mc{};
GreedyParams g_greedy{};
VoxelRemeshParams g_voxel{};
bool g_wire = true;

float g_point_pixel_size = 1.f;
glm::vec3 g_pts_viewport_color{1.f, 1.f, 1.f};
glm::vec3 g_mesh_viewport_color{1.f, 1.f, 1.f};

OrbitCamera g_cam_pts{};
OrbitCamera g_cam_mesh{};

bool g_hover_controls = false;
double g_mx = 0.0;
double g_my = 0.0;
double g_fps = 60.0;

void OnScroll(GLFWwindow*, double, double y) { g_scroll += y; }

ReconstructionMethod::Type ComboToMethod(int v) {
  switch (v) {
    case 1: return ReconstructionMethod::BallPivoting;
    case 2: return ReconstructionMethod::Poisson;
    case 3: return ReconstructionMethod::MarchingCubes;
    case 4: return ReconstructionMethod::GreedyProjection;
    default: return ReconstructionMethod::Auto;
  }
}

void EstimateNormals(PointCloud* c, std::string* log_append) {
  KdTree t(c->positions);
  normals::estimateMissing(c, t, 16, 1337);
  normals::orientConsistentTowardCenter(c);
  if (log_append) *log_append += "Oriented PCA normals rebuilt.\n";
}

void DrawPointCloudNormalsStatus() {
  size_t n_pts = 0;
  bool from_file = false;
  bool have_normals = false;
  {
    std::lock_guard lk(g_mu);
    n_pts = g_cloud.positions.size();
    from_file = g_cloud_normals_from_file;
    have_normals =
        n_pts > 0 && g_cloud.normals.size() == g_cloud.positions.size();
  }

  if (n_pts == 0) {
    ImGui::TextDisabled("Point cloud: not loaded");
    ImGui::TextDisabled("Normals: —");
    return;
  }

  ImGui::Text("Point cloud: %zu points", n_pts);
  if (!have_normals) {
    ImGui::TextColored(ImVec4(1.f, 0.4f, 0.4f, 1.f), "Normals: missing");
  } else if (from_file) {
    ImGui::TextColored(ImVec4(0.5f, 0.95f, 0.55f, 1.f),
                       "Normals: present in PLY");
  } else {
    ImGui::TextColored(ImVec4(0.95f, 0.82f, 0.4f, 1.f),
                       "Normals: not in PLY (estimated at load)");
  }
}

void HelpMarker(const char* desc) {
  ImGui::SameLine();
  ImGui::TextDisabled("(?)");
  if (ImGui::BeginItemTooltip()) {
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
    ImGui::TextUnformatted(desc);
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
  }
}

void InitAppFonts() {
  ImGuiIO& io = ImGui::GetIO();
  io.Fonts->AddFontDefault();
#if defined(_WIN32)
  g_title_font =
      io.Fonts->AddFontFromFileTTF("C:\\Windows\\Fonts\\segoeuib.ttf", 28.0f);
#elif defined(__APPLE__)
  g_title_font = io.Fonts->AddFontFromFileTTF(
      "/System/Library/Fonts/Supplemental/Arial Bold.ttf", 28.0f);
#endif
  if (!g_title_font) {
    ImFontConfig cfg;
    cfg.SizePixels = 28.0f;
    g_title_font = io.Fonts->AddFontDefault(&cfg);
  }
}

void DrawAppTitle() {
  if (g_title_font) ImGui::PushFont(g_title_font);
  ImGui::TextUnformatted(kAppName);
  if (g_title_font) ImGui::PopFont();
  ImGui::Spacing();
  ImGui::Separator();
}

JobProgress MakeJobProgress() {
  JobProgress p;
  p.report = [](float fraction, const char*) {
    g_progress.store(std::clamp(fraction, 0.f, 1.f), std::memory_order_relaxed);
  };
  return p;
}

void ResetJobProgress() {
  g_progress.store(0.f, std::memory_order_relaxed);
}

void DrawJobProgressBar() {
  if (!g_busy.load(std::memory_order_relaxed)) return;

  const float fraction = g_progress.load(std::memory_order_relaxed);
  char overlay[16];
  std::snprintf(overlay, sizeof(overlay), "%.0f%%",
                static_cast<double>(fraction) * 100.0);
  ImGui::ProgressBar(fraction, ImVec2(-1, 0), overlay);
}

void Worker(int combo_algo,
            const BPAParams& b_manual,
            const PoissonParams& p_manual,
            const MarchingCubesParams& m_manual,
            const GreedyParams& g_manual) {
  std::thread([combo_algo, b_manual, p_manual, m_manual, g_manual]() {
    ResetJobProgress();
    PointCloud local;
    {
      std::lock_guard lk(g_mu);
      local = g_cloud;
    }
    if (local.positions.empty()) {
      std::lock_guard lk(g_mu);
      g_busy = false;
      g_status = "Nothing to reconstruct (empty)";
      return;
    }

    JobProgress job = MakeJobProgress();
    job.set(0.02f, "Estimating normals");

    std::string log;
    EstimateNormals(&local, &log);

    job.set(0.05f, "Analyzing point cloud");
    CloudStats st = recon::computeStats(local, 16);
    recon::AutoSuggestion sug = recon::suggest(st);

    const auto combo = ComboToMethod(combo_algo);
    ReconstructionMethod::Type algo = combo;
    BPAParams b_run = b_manual;
    PoissonParams p_run = p_manual;
    MarchingCubesParams m_run = m_manual;
    GreedyParams g_run = g_manual;

    if (combo == ReconstructionMethod::Auto) {
      algo = sug.method;
      b_run = sug.bpa;
      p_run = sug.poisson;
      m_run = sug.marching_cubes;
      g_run = sug.greedy;
      log += "Auto-selector chose algorithm.\n";
    }

    JobProgress recon_prog = job.segment(0.08f, 0.92f);

    TriangleMesh out;
    auto t0 = std::chrono::steady_clock::now();
    switch (algo) {
      case ReconstructionMethod::BallPivoting:
        out = recon::ballPivoting(local, b_run, &recon_prog);
        log += "Ran: Ball Pivoting.\n";
        break;
      case ReconstructionMethod::Poisson:
        out = recon::poissonLikeReconstruct(local, p_run, &recon_prog);
        log += "Ran: Screened Poisson.\n";
        break;
      case ReconstructionMethod::MarchingCubes:
        out = recon::marchingCubesReconstruct(local, m_run, &recon_prog);
        log += "Ran: Marching Cubes (TSDF).\n";
        break;
      case ReconstructionMethod::GreedyProjection:
        out = recon::greedyProjectionTriangulation(local, g_run, &recon_prog);
        log += "Ran: Greedy Projection Triangulation.\n";
        break;
      default:
        log += "No algorithm dispatched.\n";
        break;
    }
    auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    {
      std::lock_guard lk(g_mu);
      g_cloud = std::move(local);
      g_mesh = std::move(out);
      g_stats = st;
      g_auto = sug;
      std::ostringstream o;
      o << log << "Triangles=" << g_mesh.indices.size()
        << ", verts=" << g_mesh.vertices.size()
        << ", time_ms=" << ms << "\n";
      g_progress.store(1.f, std::memory_order_relaxed);
      g_status = o.str();
      g_busy = false;
    }
  }).detach();
}

void WorkerVoxelRemesh(VoxelRemeshParams params) {
  std::thread([params]() {
    ResetJobProgress();
    TriangleMesh local_mesh;
    {
      std::lock_guard lk(g_mu);
      local_mesh = g_mesh;
    }
    if (local_mesh.vertices.empty() || local_mesh.indices.empty()) {
      std::lock_guard lk(g_mu);
      g_busy = false;
      g_progress.store(1.f, std::memory_order_relaxed);
      g_status.append("Fix mesh skipped (no mesh yet).\n");
      return;
    }

    JobProgress job = MakeJobProgress();
    auto t0 = std::chrono::steady_clock::now();
    TriangleMesh out =
        recon::voxelRemeshMesh(local_mesh, params, &job);
    auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    {
      std::lock_guard lk(g_mu);
      g_mesh = std::move(out);
      std::ostringstream o;
      if (g_mesh.indices.empty()) {
        o << "Fix mesh produced an empty result (try a larger voxel size).\n";
      } else {
        o << "Fix mesh complete. Triangles=" << g_mesh.indices.size()
          << ", verts=" << g_mesh.vertices.size() << ", time_ms=" << ms << "\n";
      }
      g_status.append(o.str());
      g_progress.store(1.f, std::memory_order_relaxed);
      g_busy = false;
    }
  }).detach();
}

void CursorFramebuffer(GLFWwindow* w, float* out_x, float* out_y) {
  int win_w = 1, win_h = 1, fbw = 1, fbh = 1;
  glfwGetWindowSize(w, &win_w, &win_h);
  glfwGetFramebufferSize(w, &fbw, &fbh);
  double mx = 0.0, my = 0.0;
  glfwGetCursorPos(w, &mx, &my);
  *out_x = static_cast<float>(mx) *
           static_cast<float>(fbw) / static_cast<float>(std::max(1, win_w));
  *out_y = static_cast<float>(my) *
           static_cast<float>(fbh) / static_cast<float>(std::max(1, win_h));
}

int PickFramebuffer(float fb_x, int main_w) {
  if (fb_x < 0.f || fb_x >= static_cast<float>(main_w)) return -1;
  const int half = std::max(1, main_w / 2);
  return fb_x < static_cast<float>(half) ? 0 : 1;
}

void CameraUpdate(GLFWwindow* w,
                  int main_w,
                  float r_pts,
                  glm::vec3 /*c_pts*/,
                  float r_mesh,
                  glm::vec3 /*c_mesh*/) {
  float fb_x = 0.f, fb_y = 0.f;
  CursorFramebuffer(w, &fb_x, &fb_y);

  if (g_hover_controls || main_w <= 1) {
    glfwGetCursorPos(w, &g_mx, &g_my);
  } else {
    double mx = 0.0, my = 0.0;
    glfwGetCursorPos(w, &mx, &my);
    const double dx = mx - g_mx;
    const double dy = my - g_my;
    const int vp = PickFramebuffer(fb_x, main_w);

    const float orbit_k = 0.0065f;
    const float pan_k =
        std::clamp(0.00075f * (r_pts + r_mesh + 1e-3f), 1e-5f, 0.25f);

    if (vp >= 0) {
      if (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (vp == 0)
          g_cam_pts.handleOrbit(static_cast<float>(dx), static_cast<float>(dy),
                                orbit_k);
        else
          g_cam_mesh.handleOrbit(static_cast<float>(dx), static_cast<float>(dy),
                                 orbit_k);
      }
      if (glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS ||
          glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS) {
        if (vp == 0)
          g_cam_pts.handlePan(static_cast<float>(dx), static_cast<float>(-dy),
                              pan_k);
        else
          g_cam_mesh.handlePan(static_cast<float>(dx), static_cast<float>(-dy),
                               pan_k);
      }
    }
    g_mx = mx;
    g_my = my;

    if (vp >= 0 && g_scroll != 0.0) {
      const float z = static_cast<float>(g_scroll);
      if (vp == 0) g_cam_pts.handleZoom(z, 0.12f);
      else         g_cam_mesh.handleZoom(z, 0.12f);
    }
  }
  g_scroll = 0.0;
}

void ApplyDarkGrayTheme() {
  ImGuiStyle& style = ImGui::GetStyle();
  ImVec4* c = style.Colors;
  const ImVec4 window_bg{0.12f, 0.12f, 0.12f, 1.f};
  const ImVec4 child_bg{0.10f, 0.10f, 0.10f, 1.f};
  const ImVec4 frame_bg{0.18f, 0.18f, 0.18f, 1.f};
  const ImVec4 frame_hov{0.24f, 0.24f, 0.24f, 1.f};
  const ImVec4 frame_act{0.28f, 0.28f, 0.28f, 1.f};
  const ImVec4 header{0.22f, 0.22f, 0.22f, 1.f};
  const ImVec4 header_hov{0.28f, 0.28f, 0.28f, 1.f};
  const ImVec4 header_act{0.32f, 0.32f, 0.32f, 1.f};
  const ImVec4 button{0.22f, 0.22f, 0.22f, 1.f};
  const ImVec4 button_hov{0.30f, 0.30f, 0.30f, 1.f};
  const ImVec4 button_act{0.36f, 0.36f, 0.36f, 1.f};
  const ImVec4 accent{0.72f, 0.72f, 0.72f, 1.f};

  c[ImGuiCol_Text] = ImVec4(0.90f, 0.90f, 0.90f, 1.f);
  c[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.f);
  c[ImGuiCol_WindowBg] = window_bg;
  c[ImGuiCol_ChildBg] = child_bg;
  c[ImGuiCol_PopupBg] = ImVec4(0.14f, 0.14f, 0.14f, 0.98f);
  c[ImGuiCol_Border] = ImVec4(0.28f, 0.28f, 0.28f, 1.f);
  c[ImGuiCol_FrameBg] = frame_bg;
  c[ImGuiCol_FrameBgHovered] = frame_hov;
  c[ImGuiCol_FrameBgActive] = frame_act;
  c[ImGuiCol_TitleBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.f);
  c[ImGuiCol_TitleBgActive] = ImVec4(0.14f, 0.14f, 0.14f, 1.f);
  c[ImGuiCol_TitleBgCollapsed] = ImVec4(0.10f, 0.10f, 0.10f, 0.75f);
  c[ImGuiCol_MenuBarBg] = ImVec4(0.14f, 0.14f, 0.14f, 1.f);
  c[ImGuiCol_ScrollbarBg] = ImVec4(0.10f, 0.10f, 0.10f, 1.f);
  c[ImGuiCol_ScrollbarGrab] = ImVec4(0.32f, 0.32f, 0.32f, 1.f);
  c[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.40f, 0.40f, 0.40f, 1.f);
  c[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.48f, 0.48f, 0.48f, 1.f);
  c[ImGuiCol_CheckMark] = accent;
  c[ImGuiCol_SliderGrab] = ImVec4(0.50f, 0.50f, 0.50f, 1.f);
  c[ImGuiCol_SliderGrabActive] = ImVec4(0.65f, 0.65f, 0.65f, 1.f);
  c[ImGuiCol_Button] = button;
  c[ImGuiCol_ButtonHovered] = button_hov;
  c[ImGuiCol_ButtonActive] = button_act;
  c[ImGuiCol_Header] = header;
  c[ImGuiCol_HeaderHovered] = header_hov;
  c[ImGuiCol_HeaderActive] = header_act;
  c[ImGuiCol_Separator] = ImVec4(0.30f, 0.30f, 0.30f, 1.f);
  c[ImGuiCol_SeparatorHovered] = ImVec4(0.40f, 0.40f, 0.40f, 1.f);
  c[ImGuiCol_SeparatorActive] = ImVec4(0.48f, 0.48f, 0.48f, 1.f);
  c[ImGuiCol_ResizeGrip] = ImVec4(0.35f, 0.35f, 0.35f, 0.5f);
  c[ImGuiCol_ResizeGripHovered] = ImVec4(0.45f, 0.45f, 0.45f, 0.75f);
  c[ImGuiCol_ResizeGripActive] = ImVec4(0.55f, 0.55f, 0.55f, 0.95f);
  c[ImGuiCol_Tab] = header;
  c[ImGuiCol_TabHovered] = header_hov;
  c[ImGuiCol_TabActive] = header_act;
  c[ImGuiCol_TabUnfocused] = ImVec4(0.16f, 0.16f, 0.16f, 1.f);
  c[ImGuiCol_TabUnfocusedActive] = ImVec4(0.20f, 0.20f, 0.20f, 1.f);
  c[ImGuiCol_PlotLines] = accent;
  c[ImGuiCol_PlotLinesHovered] = ImVec4(0.85f, 0.85f, 0.85f, 1.f);
  c[ImGuiCol_PlotHistogram] = accent;
  c[ImGuiCol_PlotHistogramHovered] = ImVec4(0.85f, 0.85f, 0.85f, 1.f);
  c[ImGuiCol_TextSelectedBg] = ImVec4(0.40f, 0.40f, 0.40f, 0.45f);
  c[ImGuiCol_DragDropTarget] = ImVec4(0.55f, 0.55f, 0.55f, 0.90f);
  c[ImGuiCol_NavHighlight] = ImVec4(0.65f, 0.65f, 0.65f, 1.f);
}

}

namespace app {
int Run() {
  if (!glfwInit()) return 2;
  glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
  glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);

  GLFWwindow* window =
      glfwCreateWindow(1600, 900, kAppName, nullptr, nullptr);
  if (!window) {
    glfwTerminate();
    return 3;
  }
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  glfwSetScrollCallback(window, OnScroll);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
  InitAppFonts();
  ImGui::StyleColorsDark();
  ApplyDarkGrayTheme();
  ImGui_ImplGlfw_InitForOpenGL(window, true);
  ImGui_ImplOpenGL2_Init();

  float controls_panel_w = 0.f;

  while (!glfwWindowShouldClose(window)) {
    glfwPollEvents();
    int fbw = 1600, fbh = 900;
    glfwGetFramebufferSize(window, &fbw, &fbh);

    constexpr float kMinControlsW = 280.f;
    constexpr int kMinMainAreaW = 120;
    const float max_controls_w = static_cast<float>(
        std::max(static_cast<int>(kMinControlsW), fbw - kMinMainAreaW));
    if (controls_panel_w <= 0.f) {
      controls_panel_w =
          static_cast<float>(std::clamp(fbw / 5, 360, 720));
    }
    controls_panel_w = std::clamp(controls_panel_w, kMinControlsW, max_controls_w);
    const int cw = static_cast<int>(std::lround(controls_panel_w));
    const int main_w = std::max(10, fbw - cw);

    PointCloud pts;
    TriangleMesh mesh;
    {
      std::lock_guard lk(g_mu);
      pts = g_cloud;
      mesh = g_mesh;
    }

    glm::vec3 cen_pts{0.f}, cen_mesh{0.f};
    float rad_pts = 1.f;
    float rad_mesh = 1.f;
    if (!pts.positions.empty()) render::computeBounds(pts, &cen_pts, &rad_pts);
    if (!mesh.vertices.empty()) render::computeBounds(mesh, &cen_mesh, &rad_mesh);

    CameraUpdate(window, main_w, rad_pts, cen_pts,
                 mesh.vertices.empty() ? rad_pts : rad_mesh,
                 mesh.vertices.empty() ? cen_pts : cen_mesh);

    const float scene_extent =
        glm::length(g_stats.bbox_max - g_stats.bbox_min) + 1e-6f;

    ImGui_ImplGlfw_NewFrame();
    ImGui_ImplOpenGL2_NewFrame();
    ImGui::NewFrame();

    ImGui::SetNextWindowSizeConstraints(
        ImVec2(kMinControlsW, 200.f),
        ImVec2(max_controls_w, static_cast<float>(fbh)));
    ImGuiViewport* vpp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(
        ImVec2(vpp->Pos.x + static_cast<float>(fbw - cw), vpp->Pos.y),
        ImGuiCond_Always);
    ImGui::SetNextWindowSize(
        ImVec2(controls_panel_w, static_cast<float>(fbh)),
        ImGuiCond_FirstUseEver);
    ImGui::Begin("Controls", nullptr,
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
    g_hover_controls =
        ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows);

    DrawAppTitle();

    const ImGuiIO& io_frame = ImGui::GetIO();
    if (io_frame.DeltaTime > 1e-6)
      g_fps = g_fps * 0.9 + (1.0 / io_frame.DeltaTime) * 0.1;
    ImGui::Text("FPS ~ %.1f", g_fps);
    DrawPointCloudNormalsStatus();
    ImGui::Separator();

    if (ImGui::Button("Load point cloud...", ImVec2(-1, 0))) {
      std::string path;
      if (dialogs::openPly(&path)) {
        PointCloud tmp;
        std::string err;
        if (!io::loadPly(path, &tmp, &err)) {
          g_status = std::string("PLY failed: ") + err;
        } else if (tmp.positions.empty()) {
          g_status = "PLY contained no vertices.";
        } else {
          const bool normals_from_ply =
              tmp.normals.size() == tmp.positions.size();
          EstimateNormals(&tmp, nullptr);
          CloudStats st = recon::computeStats(tmp, 16);
          recon::AutoSuggestion au = recon::suggest(st);
          {
            std::lock_guard lk(g_mu);
            g_cloud = std::move(tmp);
            g_cloud_normals_from_file = normals_from_ply;
            g_mesh = TriangleMesh{};
            g_stats = st;
            g_auto = au;

            g_bpa = au.bpa;
            g_poisson = au.poisson;
            g_mc = au.marching_cubes;
            g_greedy = au.greedy;
          }
          glm::vec3 bb_c;
          float bb_r = 1.f;
          {
            std::lock_guard lk(g_mu);
            render::computeBounds(g_cloud, &bb_c, &bb_r);
          }
          g_cam_pts.fitToSphere(bb_c, bb_r);
          g_cam_mesh.fitToSphere(bb_c, bb_r);

          std::ostringstream oss;
          oss << "Loaded: " << path << "\n"
              << "Points=" << st.point_count
              << ", normals="
              << (normals_from_ply ? "from PLY" : "estimated (no nx/ny/nz)")
              << ", median nn dist ~ " << st.median_nn_distance
              << ", density CV ~ " << st.density_cv
              << ", normal consistency ~ " << st.normal_consistency
              << "\n";
          g_status = oss.str();
        }
      }
    }

    ImGui::BeginDisabled(g_busy.load());
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 8));
    if (ImGui::Button("Generate mesh", ImVec2(-1, 44))) {
      if (!g_busy.exchange(true)) {
        ResetJobProgress();
        Worker(g_algo, g_bpa, g_poisson, g_mc, g_greedy);
      }
    }
    ImGui::PopStyleVar();
    ImGui::EndDisabled();

    DrawJobProgressBar();

    if (ImGui::Button("Export OBJ...", ImVec2(-1, 0))) {
      TriangleMesh to_save;
      {
        std::lock_guard lk(g_mu);
        to_save = g_mesh;
      }
      std::string err;
      if (to_save.indices.empty()) {
        g_status.append("No mesh available to export yet.\n");
      } else {
        std::string out;
        if (dialogs::saveObj(&out)) {
          if (io::writeObj(out, to_save, &err))
            g_status.append(std::string("Exported OBJ: ") + out + "\n");
          else
            g_status.append(std::string("OBJ export failed: ") + err + "\n");
        }
      }
    }

    ImGui::Separator();

    const char* combo_items =
        "Automatic\0"
        "Ball Pivoting\0"
        "Screened Poisson\0"
        "Marching Cubes (TSDF)\0"
        "Greedy Projection Triangulation\0";
    ImGui::Combo("Reconstruction algorithm", &g_algo, combo_items);
    HelpMarker(
        "Pick which algorithm to run.\n\n"
        "Automatic: analyse the loaded point cloud and use the "
        "best-fitting algorithm with auto-tuned parameters.\n\n"
        "Ball Pivoting: a virtual ball rolls over the cloud and forms "
        "triangles where it touches three points. Good for evenly-sampled "
        "scans.\n\n"
        "Screened Poisson: solves a Poisson equation on a uniform grid to "
        "produce a watertight surface. Best for dense, oriented clouds.\n\n"
        "Marching Cubes (TSDF): builds a Truncated SDF on a voxel grid, "
        "then extracts the zero iso-surface. Great for noisy or "
        "non-uniform clouds.\n\n"
        "Greedy Projection: walks a fringe of edges outward from a seed, "
        "stitching neighbours into triangles. Best for smooth manifolds "
        "with reasonable normals.");

    if (ImGui::Button("Apply auto-tuned parameters")) {
      CloudStats st = g_stats;
      recon::AutoSuggestion sug = recon::suggest(st);
      g_auto = sug;
      g_bpa = sug.bpa;
      g_poisson = sug.poisson;
      g_mc = sug.marching_cubes;
      g_greedy = sug.greedy;
      g_status.append(
          "Applied auto-tuned parameters from the recommender.\n");
    }
    HelpMarker(
        "Reset every algorithm's parameters to the values picked by the "
        "auto-selector for the currently loaded cloud.");

    ImGui::SameLine();
    if (ImGui::Button("Fit to view")) {
      glm::vec3 bb_c_pts{0.f}, bb_c_mesh{0.f};
      float bb_r_pts = 1.f, bb_r_mesh = 1.f;
      PointCloud snap_pts;
      TriangleMesh snap_mesh;
      {
        std::lock_guard lk(g_mu);
        snap_pts = g_cloud;
        snap_mesh = g_mesh;
      }
      if (!snap_pts.positions.empty())
        render::computeBounds(snap_pts, &bb_c_pts, &bb_r_pts);
      if (!snap_mesh.vertices.empty())
        render::computeBounds(snap_mesh, &bb_c_mesh, &bb_r_mesh);
      g_cam_pts.fitToSphere(bb_c_pts, bb_r_pts);
      g_cam_mesh.fitToSphere(
          !snap_mesh.vertices.empty() ? bb_c_mesh : bb_c_pts,
          snap_mesh.vertices.empty() ? bb_r_pts : bb_r_mesh);
    }

    ImGui::Checkbox("Show mesh wireframe", &g_wire);

    if (ImGui::CollapsingHeader("Viewport appearance")) {
      ImGui::SliderFloat("Point size (pixels)", &g_point_pixel_size, 1.f, 24.f,
                         "");
      ImGui::TextUnformatted("Point cloud colour (RGB)");
      ImGui::SliderFloat("R##vpc", &g_pts_viewport_color.r, 0.f, 1.f, "%.2f");
      ImGui::SliderFloat("G##vpc", &g_pts_viewport_color.g, 0.f, 1.f, "%.2f");
      ImGui::SliderFloat("B##vpc", &g_pts_viewport_color.b, 0.f, 1.f, "%.2f");
      ImGui::TextUnformatted("Mesh colour (RGB)");
      ImGui::SliderFloat("R##vmc", &g_mesh_viewport_color.r, 0.f, 1.f,
                         "%.2f");
      ImGui::SliderFloat("G##vmc", &g_mesh_viewport_color.g, 0.f, 1.f,
                         "%.2f");
      ImGui::SliderFloat("B##vmc", &g_mesh_viewport_color.b, 0.f, 1.f,
                         "%.2f");
    }

    ImGui::Separator();
    ImGui::TextUnformatted("Parameters");

    if (ImGui::CollapsingHeader("Ball Pivoting")) {
      ImGui::SliderFloat("Ball radius", &g_bpa.ball_radius,
                         scene_extent * 1e-5f,
                         std::max(1.f, scene_extent * 0.25f), "%.5f");
      HelpMarker(
          "Radius of the virtual ball that rolls over the cloud, in "
          "scene units. Should be roughly equal to the average distance "
          "between points.\n"
          "- Too small: the ball falls through the cloud, you get holes.\n"
          "- Too large: the ball bridges over real gaps and creates "
          "spurious triangles.");

      ImGui::SliderFloat("Radius multiplier", &g_bpa.radius_scale, 0.25f,
                         3.5f);
      HelpMarker(
          "Multiplier applied on top of 'Ball radius'. Useful if the cloud "
          "has small holes you want to bridge: scale 1.0 keeps the chosen "
          "radius, scale 1.5-2.0 fills bigger gaps.");

      ImGui::SliderFloat("Min triangle aspect", &g_bpa.min_triangle_aspect,
                         0.0005f, 0.2f, "%.4f");
      HelpMarker(
          "Reject very thin / sliver triangles below this quality "
          "threshold (4*sqrt(3)*area / perimeter^2). 0 = keep all, 0.05 = "
          "typical, >0.1 = very strict.");

      ImGui::Checkbox("Use point normals", &g_bpa.use_normals);
      HelpMarker(
          "When ON, the algorithm uses point normals to keep triangle "
          "winding consistent. Turn OFF only if your cloud has no usable "
          "normals.");
    }

    if (ImGui::CollapsingHeader("Screened Poisson")) {
      ImGui::SliderInt("Octree depth", &g_poisson.octree_depth, 4, 10);
      HelpMarker(
          "Grid resolution exponent. The grid uses 2^depth voxels along "
          "the longest bounding-box edge.\n"
          "- 6: ~64 voxels, smooth/blob-like, fast.\n"
          "- 8: ~256 voxels, balanced detail.\n"
          "- 10: ~1024 voxels, high detail but heavy on memory and time.");

      ImGui::SliderInt("Solver iterations", &g_poisson.solver_iterations,
                        10, 400);
      HelpMarker(
          "Number of Gauss-Seidel iterations used to solve the Poisson "
          "equation. More iterations = smoother surface, fewer leaks, "
          "but slower.");

      ImGui::SliderFloat("Screened weight", &g_poisson.screened_weight,
                          0.f, 4.f);
      HelpMarker(
          "Strength of the data-fidelity term that pulls the surface "
          "toward the input samples.\n"
          "- 0: pure (unscreened) Poisson - smoothest but may shrink.\n"
          "- 1: standard screened Poisson.\n"
          "- 2-3: surface hugs samples tightly (more detail, more noise).");

      ImGui::SliderFloat("Iso offset", &g_poisson.iso_level,
                          -std::max(0.05f, scene_extent * 0.01f),
                          std::max(0.05f, scene_extent * 0.01f), "%.4f");
      HelpMarker(
          "Shift the iso-value picked by the solver. Negative grows the "
          "mesh outward (helpful if the surface looks shrunken), positive "
          "shrinks it inward.");
    }

    if (ImGui::CollapsingHeader("Marching Cubes (TSDF)")) {
      ImGui::SliderInt("Grid resolution", &g_mc.grid_resolution, 32, 384);
      HelpMarker(
          "Number of voxels along the longest bounding-box edge. Higher "
          "= more detail, but cost grows as O(N^3). Typical 64-256.");

      ImGui::SliderFloat("Truncation (voxels)", &g_mc.truncation_voxels,
                          1.f, 12.f);
      HelpMarker(
          "How far (in multiples of average point spacing) to clamp the "
          "signed distance field. Smaller = sharper but more holes; "
          "larger = smoother but blobbier.");

      ImGui::SliderInt("kNN for distance", &g_mc.knn_for_distance, 1, 24);
      HelpMarker(
          "Number of nearest neighbours used per voxel to estimate the "
          "signed distance. 1 = nearest only (sharp, noisy), 8-16 = "
          "smoothed average.");

      ImGui::SliderFloat("Iso level", &g_mc.iso_level,
                          -std::max(0.05f, scene_extent * 0.05f),
                          std::max(0.05f, scene_extent * 0.05f), "%.4f");
      HelpMarker(
          "Iso-value offset for surface extraction. 0 sits exactly on the "
          "surface; negative grows it outward, positive shrinks it.");
    }

    if (ImGui::CollapsingHeader("Greedy Projection Triangulation")) {
      ImGui::SliderFloat("Search radius (x median)",
                          &g_greedy.search_radius_mult, 1.5f, 6.0f);
      HelpMarker(
          "Maximum search radius for candidate neighbours, expressed as "
          "a multiple of the median nearest-neighbour spacing. Larger = "
          "bridges bigger holes but risks spurious triangles.");

      ImGui::SliderFloat("mu (max edge factor)", &g_greedy.mu, 1.5f, 5.0f);
      HelpMarker(
          "Maximum edge length, expressed as a multiple of the distance "
          "to the nearest neighbour of the current point (PCL's mu). "
          "Typical: 2.5.");

      ImGui::SliderInt("Max neighbours", &g_greedy.max_nearest_neighbors,
                        16, 200);
      HelpMarker(
          "Maximum number of nearest neighbours considered per fringe "
          "step. Higher = more thorough but slower.");

      ImGui::SliderFloat("Max surface angle (deg)",
                          &g_greedy.max_surface_angle_deg, 10.f, 89.f);
      HelpMarker(
          "Maximum allowed angle between a candidate point's normal and "
          "the seed's normal. Filters out points lying on a different "
          "surface.");

      ImGui::SliderFloat("Min triangle angle (deg)", &g_greedy.min_angle_deg,
                          1.f, 45.f);
      HelpMarker(
          "Lower bound on triangle interior angles. Triangles with "
          "smaller angles are rejected (sliver filter).");

      ImGui::SliderFloat("Max triangle angle (deg)", &g_greedy.max_angle_deg,
                          60.f, 175.f);
      HelpMarker(
          "Upper bound on triangle interior angles. Triangles with "
          "larger (very obtuse) angles are rejected.");

      ImGui::Checkbox("Re-orient neighbour normals",
                       &g_greedy.consistent_normals);
      HelpMarker(
          "When ON, neighbour normals are flipped to agree with the "
          "current point before projection. Improves robustness on "
          "clouds with inconsistent normals.");
    }

    ImGui::BeginDisabled(g_busy.load() || mesh.indices.empty());
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(10, 7));
    if (ImGui::Button("Fix mesh", ImVec2(-1, 40))) {
      if (!g_busy.exchange(true)) {
        ResetJobProgress();
        WorkerVoxelRemesh(g_voxel);
      }
    }
    ImGui::PopStyleVar();
    ImGui::EndDisabled();

    if (ImGui::CollapsingHeader("Fix mesh settings")) {
      const float voxel_min =
          std::max(scene_extent * 1e-5f, 1e-8f);
      const float voxel_max = std::max(voxel_min, 0.5f);
      ImGui::SliderFloat("Voxel size", &g_voxel.voxel_size, voxel_min,
                         voxel_max, "%.6f");
      HelpMarker(
          "Cube edge length used to rebuild the mesh. Smaller cubes keep more "
          "detail but use more memory and take longer. Larger cubes give a "
          "smoother, coarser result.");

      ImGui::SliderInt("Hole-close passes", &g_voxel.hole_close_passes, 0,
                       24);
      HelpMarker(
          "How many times to thicken the outer shell before filling. Increase "
          "if tiny gaps let the interior leak during the fill step.");

      ImGui::SliderFloat("BBox padding (voxels)", &g_voxel.bbox_padding_voxels,
                         0.f, 24.f, "%.1f");
      HelpMarker(
          "Extra empty margin around the mesh before voxelizing. Helps avoid "
          "clipping thin parts at the bounding box edges.");

      ImGui::SliderInt("Max voxels / axis",
                       &g_voxel.max_grid_resolution_per_axis, 48, 512);
      HelpMarker(
          "Upper limit on grid width, height, and depth. If the mesh is too "
          "large for this cap, voxel size is increased automatically.");
    }

    if (ImGui::CollapsingHeader("Log")) {
      ImGui::TextWrapped("%s", g_status.c_str());
    }
    controls_panel_w = std::clamp(ImGui::GetWindowSize().x, kMinControlsW,
                                  max_controls_w);
    ImGui::End();

    render::ViewportRect left_vp{
        0, 0, std::max(4, main_w / 2), std::max(4, fbh)};
    render::ViewportRect right_vp{
        std::max(1, main_w / 2), 0, std::max(4, main_w - main_w / 2),
        std::max(4, fbh)};

    glViewport(0, 0, fbw, fbh);
    glDisable(GL_SCISSOR_TEST);
    glClearColor(0.f, 0.f, 0.f, 1.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    render::drawPointCloud(fbh, left_vp, g_cam_pts, pts,
                           g_pts_viewport_color, g_point_pixel_size);
    render::drawMesh(fbh, right_vp, g_cam_mesh, mesh,
                     g_mesh_viewport_color, g_wire);
    ImGui::Render();
    ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
    glfwSwapBuffers(window);
  }

  ImGui_ImplOpenGL2_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();
  glfwDestroyWindow(window);
  glfwTerminate();
  return 0;
}
}
