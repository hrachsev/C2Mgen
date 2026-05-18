#include "NativeFileDialogs.h"

#if defined(_WIN32)

#ifndef UNICODE
#define UNICODE
#endif

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <windows.h>

#include <string>

#include <commdlg.h>

#pragma comment(lib, "comdlg32.lib")

namespace {

std::wstring toWide(const std::string& utf8) {
  if (utf8.empty()) return L"";
  int len = MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), nullptr,
                                0);
  std::wstring out(static_cast<size_t>(len), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, utf8.data(), static_cast<int>(utf8.size()), out.data(), len);
  return out;
}

std::string toUtf8(const std::wstring& w) {
  if (w.empty()) return "";
  int len = WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), nullptr, 0,
                               nullptr, nullptr);
  std::string out(static_cast<size_t>(len), '\0');
  WideCharToMultiByte(CP_UTF8, 0, w.data(), static_cast<int>(w.size()), out.data(), len, nullptr,
                      nullptr);
  return out;
}

}

namespace dialogs {

bool openPly(std::string* out_path) {
  wchar_t szFile[MAX_PATH]{};
  OPENFILENAMEW ofn{};
  ofn.lStructSize = sizeof(ofn);
  ofn.lpstrFilter = L"PLY Files (*.ply)\0*.PLY\0All Files (*.*)\0*.*\0";
  ofn.lpstrFile = szFile;
  ofn.nMaxFile = MAX_PATH;
  ofn.lpstrTitle = L"Open point cloud (.ply)";
  ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_EXPLORER;
  const BOOL ok = GetOpenFileNameW(&ofn);
  if (!ok) return false;
  if (out_path) *out_path = toUtf8(szFile);
  return true;
}

bool saveObj(std::string* out_path) {
  wchar_t szFile[MAX_PATH]{};
  OPENFILENAMEW ofn{};
  ofn.lStructSize = sizeof(ofn);
  ofn.lpstrFilter = L"OBJ Wavefront (*.obj)\0*.OBJ\0All Files (*.*)\0*.*\0";
  ofn.lpstrFile = szFile;
  ofn.nMaxFile = MAX_PATH;
  ofn.lpstrTitle = L"Export mesh (.obj)";
  ofn.Flags = OFN_OVERWRITEPROMPT | OFN_EXPLORER;
  const BOOL ok = GetSaveFileNameW(&ofn);
  if (!ok) return false;
  std::wstring w = szFile;
  if (w.size() < 4) {
    w += L".obj";
  } else {
    const std::wstring tail = w.substr(w.size() - 4);
    if (!(tail == L".obj" || tail == L".OBJ")) w += L".obj";
  }
  if (out_path) *out_path = toUtf8(w);
  return true;
}

}

#else

#include <iostream>

namespace dialogs {

bool openPly(std::string* out_path) {
  std::cout << "Path to .ply: " << std::flush;
  std::string tmp;
  if (!(std::getline(std::cin, tmp))) return false;
  if (!tmp.empty()) *out_path = tmp;
  return !out_path || !out_path->empty();
}

bool saveObj(std::string* out_path) {
  std::cout << "Path to export .obj: " << std::flush;
  std::string tmp;
  if (!(std::getline(std::cin, tmp))) return false;
  if (!tmp.empty()) *out_path = tmp;
  return !out_path || !out_path->empty();
}

}

#endif
