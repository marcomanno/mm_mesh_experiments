#include "Boolean/boolean.hh"
#include "Import/import.hh"

int main(int _argc, const char* _argv[])
{
  if (_argc < 4)
    return 1;
  auto body0 = IO::load_obj(_argv[1]);
  auto body1 = IO::load_obj(_argv[2]);
  auto op = Boolean::OperationHelper::to_enum(_argv[3]);
  if (op == Boolean::Operation::ENUM_SIZE)
    return 1;
  auto bool_sol = Boolean::ISolver::make();
  bool_sol->init(body0, body1);
  auto res = bool_sol->compute(op);
  IO::save_obj("result.obj", res);
  return 0;
}