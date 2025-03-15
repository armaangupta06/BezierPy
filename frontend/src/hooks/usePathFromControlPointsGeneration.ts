import { useState } from 'react';
import { useMutation, QueryClient } from '@tanstack/react-query';
import apiService, { BezierCurveModel, CreatePathFromControlPointsRequest, PathResponse } from '@/services/api';
import bezierService from '@/services/bezierService';

interface UsePathFromControlPointsGenerationProps {
  queryClient: QueryClient;
  onSuccess?: (pathId: string, points: any[], curves?: any[]) => void;
  onError?: (error: Error) => void;
}

export default function usePathFromControlPointsGeneration({
  queryClient,
  onSuccess,
  onError,
}: UsePathFromControlPointsGenerationProps) {
  const [isLoading, setIsLoading] = useState(false);

  const generatePathMutation = useMutation<PathResponse, Error, CreatePathFromControlPointsRequest>({
    mutationFn: async (data: CreatePathFromControlPointsRequest) => {
      try {
        // Use local bezierService instead of API
        const response = bezierService.createPathFromControlPoints(data);
        return Promise.resolve(response);
      } catch (error) {
        console.error('Error in local path generation:', error);
        throw error;
      }
    },
    onSuccess: (data) => {
      if (onSuccess) {
        onSuccess(data.path_id, data.discretized_points || [], data.curves || []);
      }
      // Invalidate queries that might be affected
      queryClient.invalidateQueries({ queryKey: ['path', data.path_id] });
    },
    onError: (error: Error) => {
      console.error('Error generating path:', error);
      if (onError) {
        onError(error);
      }
    },
    onSettled: () => {
      setIsLoading(false);
    },
  });

  const generatePath = async (controlPointsList: BezierCurveModel[]) => {
    if (controlPointsList.length < 1) {
      throw new Error('At least one Bezier curve is required to generate a path');
    }

    // Validate that each curve has exactly 6 control points
    for (const curve of controlPointsList) {
      if (curve.control_points.length !== 6) {
        throw new Error('Each Bezier curve must have exactly 6 control points');
      }
    }

    setIsLoading(true);
    return generatePathMutation.mutate({
      control_points_list: controlPointsList,
    });
  };

  return {
    generatePath,
    isLoading,
  };
}
