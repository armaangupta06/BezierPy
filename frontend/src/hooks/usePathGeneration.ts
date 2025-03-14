import { useState } from 'react';
import { useMutation, useQuery, QueryClient } from '@tanstack/react-query';
import apiService, { 
  PoseModel,
  CreatePathFromPointsRequest, 
  CreatePathFromPosesRequest, 
  PathResponse 
} from '@/services/api';

interface UsePathGenerationProps {
  queryClient: QueryClient;
  onSuccess?: (pathId: string, points: any[], curves?: any[]) => void;
  onError?: (error: Error) => void;
}

/**
 * Hook for path generation operations
 */
export default function usePathGeneration({
  queryClient,
  onSuccess,
  onError,
}: UsePathGenerationProps) {
  const [isLoading, setIsLoading] = useState(false);

  // Create path from poses
  const createPathFromPosesMutation = useMutation({
    mutationFn: (data: CreatePathFromPosesRequest) => 
      apiService.createPathFromPoses(data),
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

  // Get path by ID
  const getPath = (pathId: string | null, includeDiscretized: boolean = true) => {
    return useQuery({
      queryKey: ['path', pathId, includeDiscretized],
      queryFn: () => apiService.getPath(pathId!, includeDiscretized),
      enabled: !!pathId, // Only run query if pathId is provided
    });
  };

  // Delete path
  const deletePath = useMutation({
    mutationFn: (pathId: string) => 
      apiService.deletePath(pathId),
  });

  // Generate path from poses
  const generatePath = async (poses: PoseModel[], tangentMagnitude: number = 0.8) => {
    if (poses.length < 2) {
      throw new Error('At least 2 poses are required to generate a path');
    }

    setIsLoading(true);
    return createPathFromPosesMutation.mutate({
      poses,
      params: {
        tangent_magnitude: tangentMagnitude,
      },
    });
  };

  return {
    generatePath,
    isLoading,
    getPath,
    deletePath,
  };
};


