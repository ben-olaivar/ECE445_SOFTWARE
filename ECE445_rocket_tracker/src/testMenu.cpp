#include <iostream>
#include <string>
#include <vector>

int menu_type = 0;
int main_m_row_num = 0;
int set_freq_row_num = 1;
int t_frequency = 0;
int LOW = 0;

void set_frequency(int frequency) {
    t_frequency = frequency;
}


void print_menu(int m_type) {
  std::cout << "" << std::endl;
  std::cout << "--------------MENU--------------" << std::endl;
  //Serial.println("--------------MENU--------------");

  switch (m_type) {
    case 0: // This is main menu selected Compass
        std::cout << "> Compass <" << std::endl;
        std::cout << "Frequency Change" << std::endl;
        break;
    case 1: // Main menu selected Frequency Change
        std::cout << "Compass" << std::endl;
        std::cout << "> Frequency Change <" << std::endl;
        break;  
    case 2: // This is sub menu of selected Compass
        std::cout << "Relevant Information:" << std::endl;
        std::cout << "> Back  <" << std::endl; // this to mimic enter button
        break;
    case 3: // This is sub menu of selected Frequency Change
        std::cout << t_frequency << std::endl;
        std::cout << "> Set Frequency: <" << std::endl;
        std::cout << " Back " << std::endl;
        break;  
    case 4: // This is sub menu of selected Frequency Change
        std::cout << t_frequency << std::endl;
        std::cout << "Set Frequency:" << std::endl;
        std::cout << "> Back <" << std::endl;
        break;  
    case 5: // This is sub menu of Set Frequency Change
        std::cout << "Set Frequency:" << std::endl;
        break;
    default:
        break;
  }
}

void executeAction(){
    switch (menu_type) {
        case 0: // user pressed Enter on Compass
            // set the menu type to sub menu of selected Compass
            menu_type = 2;
            print_menu(menu_type);
            break;
        case 1: // User Pressed Enter on Frequency Change
            // set the menu type to sub menu of selected Frequency Change
            menu_type = 3;
            print_menu(menu_type);
            break;  
        case 2: // This is sub menu of selected Compass
            menu_type = 0; // go back to main menu
            print_menu(menu_type);
            break;
        case 3: // This is sub menu of selected Frequency Change
            menu_type = 5; // set frequency
            print_menu(menu_type);
            break;  
        case 4: // User pressed the back button from set frequency
            menu_type = 1; // go back to main menu
            print_menu(menu_type);
            break;
        case 5: // user entered the frequency & pressed enter to set the frequency
            set_frequency(t_frequency);
            menu_type = 4; // go back to set frequency
            print_menu(menu_type);
            break;
        default:
            break;
    }
}

void loop(int d_state, int u_state, int e_state) {
  // Serial.println(menu_state);
  // read the state of the pushbutton value:
  int down_button_state = d_state; //digitalRead(down_button_pin);
  int up_button_state   = u_state; //digitalRead(up_button_pin);
  int enter_button_state = e_state; //digitalRead(enter_button_pin);


  // check if the pushbutton is pressed. If it is, the down_button_state is HIGH:
    if (down_button_state != LOW) {
        down_button_state = LOW;
        if ((menu_type == 0 || menu_type == 1) && main_m_row_num == 0){ // if we are at the main menu and the first row
            main_m_row_num += 1;
            menu_type = 1; // main menu has two menu 0 & 1 - 1 to highlight the Frequency Change 
            print_menu(menu_type);
        } else if ((menu_type == 3 || menu_type == 4) && set_freq_row_num == 1) {
            set_freq_row_num += 1;
            menu_type = 4; //menu type 3 & 4 to set frequency - 4 is to highlight the back button
            print_menu(menu_type);
        }
    } else if (up_button_state != LOW) {
        up_button_state = LOW;
        if ((menu_type == 0 || menu_type == 1) && main_m_row_num == 1){ // if we are at the main menu and the second row
            main_m_row_num -= 1;
            menu_type = 0; // main menu has two menu 0 & 1 - 0 to highlight Compass
            print_menu(menu_type);
        } else if ((menu_type == 3 || menu_type == 4) && set_freq_row_num == 2) {
            set_freq_row_num -= 1;
            menu_type = 3; //menu type 3 & 4 to set frequency - 3 is to highlight set frequency
            print_menu(menu_type);
        }
    } else if (enter_button_state != LOW) {
        enter_button_state = LOW;
        executeAction();

    } 
}

// main method to set the frequency
int main() {
    // set the frequency
    set_frequency(100);
    // set the menu type to sub menu of selected Frequency Change
    menu_type = 0;
    // print the menu
    print_menu(menu_type);
    // loop
    std::cout << " Press Down Arrow " << std::endl;
    loop(1,0,0);
    std::cout << " Pres Up Arrow " << std::endl;
    loop(0,1,0);
    std::cout << "Press Enter " << std::endl;
    loop(0,0,1);
    std::cout << "Press Enter " << std::endl;
    loop(0,0,1);
    std::cout << "Press Down Arrow " << std::endl;
    loop(1,0,0);
    std::cout << "Press Enter " << std::endl;
    loop(0,0,1);
    std::cout << "Press Enter " << std::endl;
    loop(0,0,1);
    std::cout << "Press Enter " << std::endl;
    loop(0,0,1);
    std::cout << "Press Down Arrow " << std::endl;
    loop(1,0,0);
    std::cout << "Press Enter " << std::endl;
    loop(0,0,1);
    return 0;
}   

