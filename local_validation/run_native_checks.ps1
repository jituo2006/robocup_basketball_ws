param([string]$FirmwareRoot = 'E:/Dji/dji_driver_board-feature-cxx')
$ErrorActionPreference = 'Stop'
$robotRoot = Split-Path $PSScriptRoot -Parent
Push-Location $robotRoot
try {
  $robotWrap = '-Wl,--wrap=socket,--wrap=bind,--wrap=ioctl,--wrap=setsockopt,--wrap=send,--wrap=recv,--wrap=poll,--wrap=close,--wrap=write'
  & g++ -std=c++17 -Wall -Wextra -Werror -static -I local_validation/include -I src/rb_chassis/third_party -I src/rb_chassis/include src/rb_chassis/third_party/bupt_can.cpp src/rb_chassis/test/test_can_transport.cpp $robotWrap -o local_validation/test_can_transport.exe
  if ($LASTEXITCODE) { throw 'CAN transport compile failed' }
  & ./local_validation/test_can_transport.exe
  if ($LASTEXITCODE) { throw 'CAN transport test failed' }
  & g++ -std=c++17 '-DM_PI=3.14159265358979323846' -Wall -Wextra -Werror -static -I local_validation/controller_include -I src/rb_chassis/include local_validation/test_chassis_control.cpp src/rb_chassis/src/base_chassis.cpp src/rb_chassis/src/omni_chassis.cpp -o local_validation/test_chassis_control.exe
  if ($LASTEXITCODE) { throw 'Controller compile failed' }
  & ./local_validation/test_chassis_control.exe
  if ($LASTEXITCODE) { throw 'Controller test failed' }
  & g++ -std=c++17 -Wall -Wextra -Werror -static -I local_validation/include -I src/rb_chassis/include src/rb_chassis/test/test_board_link.cpp -o local_validation/test_board_link.exe
  if ($LASTEXITCODE) { throw 'Board link compile failed' }
  & ./local_validation/test_board_link.exe
  if ($LASTEXITCODE) { throw 'Board link test failed' }
  & python -X utf8 local_validation/run_protocol_tests.py $robotRoot $FirmwareRoot
  if ($LASTEXITCODE) { throw 'Paired firmware protocol test failed' }
} finally { Pop-Location }
