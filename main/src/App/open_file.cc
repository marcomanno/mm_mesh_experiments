
#include <open_file.hh>

#include <windows.h>
#include <Commdlg.h>

#include <algorithm>


std::string open_file()
{
  OPENFILENAME ofn;
  // a another memory buffer to contain the file name
  char szFile[100];
  memset(&ofn, 0, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = NULL;
  ofn.lpstrFile = szFile;
  ofn.lpstrFile[0] = '\0';
  ofn.nMaxFile = sizeof(szFile);
  ofn.lpstrFilter = "OBJ\0*.OBJ\0OBJ\0*.obj\0";
  ofn.nFilterIndex = 1;
  ofn.lpstrFileTitle = NULL;
  ofn.nMaxFileTitle = 0;
  ofn.lpstrInitialDir = "C:\\Users\\marco\\OneDrive\\Documents\\PROJECTS\\polytriagnulation\\mesh";
  ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;
  GetOpenFileName(&ofn);
  return ofn.lpstrFile;
}