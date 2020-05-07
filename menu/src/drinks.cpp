#include "pugixml.hpp"
#include <iostream>
#include <sstream>

int main()
{
    pugi::xml_document doc;
    std::string namePanel;

    std::cout << "Test" << std::endl;

    if (!doc.load_file("../drinks.xml")) return -1;

    std::cout << "Test" << std::endl;

    pugi::xml_node drinks = doc.child("menu").child("drink");

    std::cout << "Drink: " << drinks.attribute("name").value() << " : " << drinks.child("name").text().get() << std::endl;
    std::cout << "Drink: " << drinks.child_value("name") << std::endl;


    return 0;
}