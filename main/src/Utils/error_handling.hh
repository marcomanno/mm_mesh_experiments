
#pragma once

#define THROW(MESSAGE) { throw MESSAGE; }

#define THROW_IF(COND, MESSAGE) if (COND) THROW(MESSAGE);
