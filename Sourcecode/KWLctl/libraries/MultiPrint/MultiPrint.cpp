/*
 * Copyright (C) 2018 Ivan Schréter (schreter@gmx.net)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * This copyright notice MUST APPEAR in all copies of the software!
 */

#include "MultiPrint.h"

size_t MultiPrint::write(uint8_t c)
{
  if (!s1_)
    return 0;
  auto res = s1_->write(c);
  if (s2_)
    s2_->write(c);
  setWriteError(s1_->getWriteError());
  return res;
}

size_t MultiPrint::write(const uint8_t *buffer, size_t size)
{
  if (!s1_)
    return 0;
  auto res = s1_->write(buffer, size);
  if (s2_)
    s2_->write(buffer, size);
  setWriteError(s1_->getWriteError());
  return res;
}

int MultiPrint::availableForWrite()
{
  if (!s1_)
    return 0;
  return s1_->availableForWrite();
}

void MultiPrint::flush()
{
  if (s1_)
    s1_->flush();
  if (s2_)
    s2_->flush();
}
