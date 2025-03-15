import { useState } from 'react';
import { useMutation, QueryClient } from '@tanstack/react-query';
import apiService, { PointModel, CreatePathFromPointsRequest, PathResponse } from '@/services/api';
import bezierService from '@/services/bezierService';

interface UsePathFromPointsGenerationProps {
  queryClient: QueryClient;
  onSuccess?: (pathId: string, points: any[], curves?: any[]) => void;
  onError?: (error: Error) => void;
}

export default function usePathFromPointsGeneration({
  queryClient,
  onSuccess,
  onError,
}: UsePathFromPointsGenerationProps) {
  const [isLoading, setIsLoading] = useState(false);

  const generatePathMutation = useMutation<PathResponse, Error, CreatePathFromPointsRequest>({
    mutationFn: async (data: CreatePathFromPointsRequest) => {
      try {
        // Use local bezierService instead of API
        const response = bezierService.createPathFromPoints(data);
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

  const generatePath = async (
    points: PointModel[],
    initialHeading: number,
    finalHeading?: number,
    tangentMagnitude: number = 0.8
  ) => {
    if (points.length < 2) {
      throw new Error('At least 2 points are required to generate a path');
    }

    setIsLoading(true);
    return generatePathMutation.mutate({
      points,
      initial_heading: initialHeading,
      final_heading: finalHeading,
      params: {
        tangent_magnitude: tangentMagnitude,
      },
    });
  };

  return {
    generatePath,
    isLoading,
  };
}
