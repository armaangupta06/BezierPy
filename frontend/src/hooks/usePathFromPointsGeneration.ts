import { useState } from 'react';
import { useMutation, QueryClient } from '@tanstack/react-query';
import apiService, { PointModel, CreatePathFromPointsRequest } from '@/services/api';

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

  const generatePathMutation = useMutation({
    mutationFn: (data: CreatePathFromPointsRequest) => {
      return apiService.createPathFromPoints(data);
    },
    onSuccess: async (data) => {
      // Get discretized points
      try {
        const pathResponse = await apiService.getPath(data.path_id, true);
        if (onSuccess) {
          onSuccess(data.path_id, pathResponse.discretized_points || [], pathResponse.curves || []);
        }
        // Invalidate queries that might be affected
        queryClient.invalidateQueries({ queryKey: ['path', data.path_id] });
      } catch (error) {
        console.error('Error fetching path details:', error);
        if (onError) {
          onError(error instanceof Error ? error : new Error('Unknown error'));
        }
      }
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
