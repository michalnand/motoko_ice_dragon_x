#include <drivers.h>

#include <path_planner.h>

#include <LineFollowing.h>


int main()
{
    drivers_init(); 
  
    terminal << "\n\n\n"; 
    terminal << "machine ready\n";
    terminal << "system clock " << SystemCoreClock/1000000 << "MHz\n";


    LineFollowing robot;
    robot.init();

    // wait for key press
    int key_result = button();

    while (true)
    {
      robot.main();
      //robot.line_search(LINE_LOST_CENTER, 0);
    }

    return 0;
}
  