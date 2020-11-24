/*
 * Copyright (C) OpenTX
 *
 * Source:
 *  https://github.com/opentx/libopenui
 *
 * This file is a part of libopenui library.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 */

#pragma once

#include "window.h"

class Layer
{
  public:
    explicit Layer(Window * main):
      main(main)
    {
    }

    Window * main;
    Window * focus = nullptr;

    static std::list<Layer> stack;

    static void push(Window * window)
    {
      if (!stack.empty()) {
        Layer &parentLayer = stack.back();
        parentLayer.focus = Window::getFocus();
      }
      stack.emplace_back(Layer(window));
    }

    static void pop(Window * window)
    {
      if (stack.back().main == window) {
        stack.pop_back();
        const auto & back = stack.back();
        if (back.focus) {
          back.focus->setFocus(SET_FOCUS_DEFAULT);
        }
      }
      else {
        // TODO it would be better to do it in reverse order
        for (auto it = stack.begin(); it != stack.end(); it++) {
          if (it->main == window) {
            stack.erase(it);
            break;
          }
        }
      }
    }
};
