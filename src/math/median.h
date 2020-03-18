#ifndef __MEDIAN_TEMPLATE__
#define __MEDIAN_TEMPLATE__

// Iteratively calculates median for up to SIZE data, and ignores the rest after
// overflow.
//
// Useful for interrupt-driven data fill, to minimize possible locks.
// Worst case ~ scan of 1/2 collected elements.
//
// How it works (in general): https://stackoverflow.com/a/15319593/1031804

template <typename T, int SIZE>
class MedianIteratorTemplate {

public:
    MedianIteratorTemplate() {
      reset();
    }

    void reset()
    {
        heap_lo_len = 0;
        heap_hi_len = 0;
        heap_lo_max_idx = 0;
        heap_hi_min_idx = 0;
    }

    T result()
    {
        int total_len = heap_lo_len + heap_hi_len;

        // If not enougth data, create result manually
        switch (total_len){
            case 0: return 0;
            case 1: return heap_lo[0];
            case 2: return (heap_lo[0] + heap_hi[0]) / 2;
        }

        if (total_len & 0x01)
        {
            return (heap_lo_len > heap_hi_len) ? heap_lo[heap_lo_max_idx] : heap_hi[heap_hi_min_idx];
        }
        return (heap_lo[heap_lo_max_idx] + heap_hi[heap_hi_min_idx]) / 2;
    }

    void add(T val) {
        // Process special cases
        switch (heap_lo_len + heap_hi_len) {
            case 0:
                // First data => push to low heap
                heap_lo[0] = val;
                heap_lo_len = 1;
                return;

            case 1:
                // Second data => low and high heaps should contain per 1 element each
                // at the end. Reorder if needed.
                if (val < heap_lo[0])
                {
                  heap_hi[0] = heap_lo[0];
                  heap_lo[0] = val;
                }
                else heap_hi[0] = val;

                heap_hi_len = 1;
                return;

            case SIZE:
                // If all buffers occupied - stop accepting new data
                return;
        }

        // Now we have enough to start with safe min/max searches.
        T lo_max_val = heap_lo[heap_lo_max_idx];
        T hi_min_val = heap_hi[heap_hi_min_idx];

        if (val < lo_max_val)
        {
            // New value must go to low heap
            if (heap_lo_len > heap_hi_len)
            {
                // Ups... curent size "too big" => transfer existing max to high heap
                // (and put new value to it's place)
                heap_hi[heap_hi_len] = lo_max_val;
                heap_hi_min_idx = heap_hi_len;
                heap_hi_len++;

                // Need refresh max after such update
                heap_lo[heap_lo_max_idx] = val;
                heap_lo_max_idx = get_heap_lo_max_idx();
            }
            else
            {
                // Just add value to low heap
                heap_lo[heap_lo_len++] = val;
            }
        }
        else if (val >= hi_min_val)
        {
            // New value must go to high heap
            if (heap_hi_len > heap_lo_len)
            {
                // Ups... curent size "too big" => transfer existing min to low heap
                // (and put new value to it's place)
                heap_lo[heap_lo_len] = hi_min_val;
                heap_lo_max_idx = heap_lo_len;
                heap_lo_len++;

                // Need refresh min after such update
                heap_hi[heap_hi_min_idx] = val;
                heap_hi_min_idx = get_heap_hi_min_idx();
            }
            else
            {
                // Just add value to heap_hi
                heap_hi[heap_hi_len++] = val;
            }
        }
        else
        {
            // Value is somewhere "between" heaps => try push to lower heap,
            // if it's size is not too big
            if (heap_lo_len > heap_hi_len)
            {
                // No space in low heap => push to high one
                heap_hi[heap_hi_len] = val;
                heap_hi_min_idx = heap_hi_len;
                heap_hi_len++;
            }
            else
            {
                heap_lo[heap_lo_len] = val;
                heap_lo_max_idx = heap_lo_len;
                heap_lo_len++;
            }
        }
    }

private:
    T heap_lo[SIZE / 2 + (SIZE & 1)];
    int heap_lo_len;
    int heap_lo_max_idx;

    T heap_hi[SIZE / 2 + (SIZE & 1)];
    int heap_hi_len;
    int heap_hi_min_idx;

    int get_heap_lo_max_idx()
    {
        int i, idx;
        i = idx = heap_lo_len - 1;
        T val = heap_lo[i];

        while (i--) {
            int tmp = heap_lo[i];
            if (tmp > val) { val = tmp; idx = i; }
        }

        return idx;
    }

    int get_heap_hi_min_idx()
    {
        int i, idx;
        i = idx = heap_hi_len - 1;
        T val = heap_hi[i];

        while (i--) {
            int tmp = heap_hi[i];
            if (tmp < val) { val = tmp; idx = i; }
        }

        return idx;
    }
};

#endif
