"""
segmentation.py — Depth-based ground plane and obstacle segmentation.

Provides the `DepthSegmenter` class, which calibrates a ground plane from
initial depth frames using RANSAC, and then segments subsequent frames to
find obstacles (objects sticking up from the ground).
"""

import logging
from typing import List, Optional, Tuple, Dict, Any

import cv2
import numpy as np

logger = logging.getLogger(__name__)


class DepthSegmenter:
    """Segments a depth stream into ground and obstacles.
    
    The segmenter must be calibrated with one or more frames of the empty 
    arena to calculate the ground plane equation. Once calibrated, it can 
    find obstacles in real-time.
    """

    def __init__(
        self,
        max_depth_mm: int = 3500,
        obstacle_min_height_mm: float = 25.0,
        obstacle_min_area_px: int = 100,
    ) -> None:
        self.max_depth_mm = max_depth_mm
        self.obstacle_min_height_mm = obstacle_min_height_mm
        self.obstacle_min_area_px = obstacle_min_area_px

        self._plane_coeffs: Optional[Tuple[float, float, float]] = None
        self._predicted_ground: Optional[np.ndarray] = None
        
        # Morphological kernel for cleaning up obstacle masks
        self._kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))

    @property
    def is_calibrated(self) -> bool:
        """Returns True if the ground plane has been calibrated."""
        return self._plane_coeffs is not None

    def calibrate(self, depth_frames: List[np.ndarray], mask: Optional[np.ndarray] = None, iterations: int = 500, inlier_threshold_mm: float = 15.0) -> bool:
        """Calibrate the ground plane using RANSAC on a set of depth frames.
        
        Averages the provided frames to reduce noise, then fits a plane 
        equation: depth = a*col + b*row + c.
        If `mask` is provided, only pixels where mask > 0 are used.
        
        Returns True if successful, False otherwise.
        """
        if not depth_frames:
            return False

        # Average frames to reduce noise
        depth_acc = np.zeros_like(depth_frames[0], dtype=np.float64)
        depth_count = np.zeros_like(depth_frames[0], dtype=np.int32)

        for d in depth_frames:
            valid = d > 0
            depth_acc[valid] += d[valid]
            depth_count[valid] += 1

        with np.errstate(divide='ignore', invalid='ignore'):
            depth_avg = np.where(depth_count > 0, depth_acc / depth_count, 0).astype(np.float32)

        h, w = depth_avg.shape
        valid = (depth_avg > 0) & (depth_avg < self.max_depth_mm)
        if mask is not None:
            valid = valid & (mask > 0)
            
        rows, cols = np.where(valid)
        depths = depth_avg[valid]

        n_valid = len(depths)
        if n_valid < 100:
            logger.warning("Too few valid depth pixels (%d) for plane fitting", n_valid)
            return False

        best_inliers = 0
        best_plane = None

        for _ in range(iterations):
            # Pick 3 random points
            idx = np.random.choice(n_valid, 3, replace=False)
            r = rows[idx].astype(np.float64)
            c = cols[idx].astype(np.float64)
            
            # Inverse depth!
            inv_d = 1.0 / depths[idx].astype(np.float64)

            # Solve: 1/d = a*c + b*r + const  →  [c r 1] @ [a b const]^T = 1/d
            A = np.column_stack([c, r, np.ones(3)])
            try:
                plane = np.linalg.solve(A, inv_d)
            except np.linalg.LinAlgError:
                continue

            a, b, const = plane

            # Compute residuals
            predicted_inv = a * cols.astype(np.float64) + b * rows.astype(np.float64) + const
            # Avoid division by zero if plane intersects 0
            valid_pred = predicted_inv > 1e-6
            
            inlier_count = 0
            if np.any(valid_pred):
                predicted = 1.0 / predicted_inv[valid_pred]
                residuals = np.abs(depths.astype(np.float64)[valid_pred] - predicted)
                inlier_count = np.sum(residuals < inlier_threshold_mm)

            if inlier_count > best_inliers:
                best_inliers = inlier_count
                best_plane = plane

        if best_plane is None:
            logger.error("RANSAC failed to find a ground plane")
            return False

        self._plane_coeffs = tuple(best_plane)
        
        # Precompute the predicted ground depth map for fast real-time subtraction
        a, b, c = self._plane_coeffs
        predicted_ground_inv = a * np.arange(w)[None, :] + b * np.arange(h)[:, None] + c
        # Clip to avoid division by zero
        predicted_ground_inv = np.clip(predicted_ground_inv, 1e-6, np.inf)
        self._predicted_ground = 1.0 / predicted_ground_inv
        
        logger.info("Ground plane (1/Z) calibrated: 1/depth = %.2e*col + %.2e*row + %.2e", a, b, c)
        logger.info("RANSAC inliers: %d / %d (%.1f%%)", best_inliers, n_valid, 100 * best_inliers / n_valid)
        return True

    def find_obstacles(self, depth: np.ndarray) -> List[Dict[str, Any]]:
        """Find obstacles in the given depth frame.
        
        Requires calibration to be performed first.
        
        Returns a list of obstacle dicts:
            [
                {
                    "bbox": (x, y, w, h),
                    "area": int,
                    "centroid": (cx, cy),
                    "mean_height_mm": float,
                }, ...
            ]
        """
        if not self.is_calibrated:
            logger.warning("DepthSegmenter is not calibrated, cannot find obstacles")
            return []

        h, w = depth.shape
        valid = (depth > 0) & (depth < self.max_depth_mm)

        # "Above the ground" means closer to the camera -> depth < predicted - threshold.
        # We also need to ignore zero depths and depths beyond the max range.
        above_mask = valid & (depth < self._predicted_ground - self.obstacle_min_height_mm)

        # Morphological cleanup
        above_uint8 = above_mask.astype(np.uint8) * 255
        above_uint8 = cv2.morphologyEx(above_uint8, cv2.MORPH_OPEN, self._kernel)
        above_uint8 = cv2.morphologyEx(above_uint8, cv2.MORPH_CLOSE, self._kernel)

        contours, _ = cv2.findContours(above_uint8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        obstacles = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < self.obstacle_min_area_px:
                continue
                
            x, y, bw, bh = cv2.boundingRect(cnt)
            
            # Calculate mean height
            mask_roi = above_uint8[y:y+bh, x:x+bw] > 0
            depth_roi = depth[y:y+bh, x:x+bw]
            pred_roi = self._predicted_ground[y:y+bh, x:x+bw]
            
            # heights = ground - actual
            heights = pred_roi[mask_roi] - depth_roi[mask_roi]
            mean_height = float(np.mean(heights)) if len(heights) > 0 else 0.0

            # Calculate centroid
            M = cv2.moments(cnt)
            cx = int(M["m10"] / M["m00"]) if M["m00"] > 0 else x + bw // 2
            cy = int(M["m01"] / M["m00"]) if M["m00"] > 0 else y + bh // 2

            obstacles.append({
                "bbox": (x, y, bw, bh),
                "area": int(area),
                "centroid": (cx, cy),
                "mean_height_mm": mean_height,
            })

        # Sort largest to smallest
        obstacles.sort(key=lambda o: o["area"], reverse=True)
        return obstacles

    def get_ground_mask(self, depth: np.ndarray, threshold_mm: float = 20.0) -> np.ndarray:
        """Returns a boolean mask of pixels belonging to the ground plane."""
        if not self.is_calibrated:
            return np.zeros_like(depth, dtype=bool)
            
        valid = (depth > 0) & (depth < self.max_depth_mm)
        residual = np.abs(depth.astype(np.float64) - self._predicted_ground)
        return valid & (residual < threshold_mm)
