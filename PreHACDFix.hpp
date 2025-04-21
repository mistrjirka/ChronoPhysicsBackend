#pragma once
// Declare HeapManager in the global namespace as a dummy type.
class HeapManager {};

// In the HACD namespace, forward-declare ICHull and alias ICHUll.
namespace HACD {
    class ICHull;  // forward declaration
    typedef ICHull ICHUll;
}
