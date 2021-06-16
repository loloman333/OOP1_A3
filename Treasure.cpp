//----------------------------------------------------------------------------------------------------------------------
// Treasures.cpp
//
// The treasure class represent a Treasure.
// Players need to collect these treasures to win the game.
//
// Authors: Triochter Bande (Grill Matthias, Killer Lorenz, Nagy Lukas)
//----------------------------------------------------------------------------------------------------------------------
//

#include "Treasure.hpp"

Treasure::Treasure(std::string name, size_t treasure_id) : name_{name}, treasure_id_{treasure_id} {}

std::string Treasure::getName()
{
  return name_;
}

size_t Treasure::getTreasureId()
{
  return treasure_id_;
}

bool Treasure::getFound()
{
  return found_;
}

void Treasure::setFound(bool found)
{
  found_ = found;
}
