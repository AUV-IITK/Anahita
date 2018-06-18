#ifndef blue_filter_H
#define blue_filter_H

#include "opencv2/core/core.hpp"

namespace vision_commons {
	class BlueFilter {
		public:
			static cv::Mat filter(
				cv::Mat image,
				double clahe_clip,
				int clahe_grid_size,
				int clahe_bilateral_iter,
				int balanced_bilateral_iter,
				double denoise_h
			);
	};
}

#endif
