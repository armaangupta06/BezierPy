import { useState } from 'react';
import { useMutation, useQuery, QueryClient } from '@tanstack/react-query';
import apiService, { 
  PoseModel,
  CreatePathFromPointsRequest, 
  CreatePathFromPosesRequest, 
  PathResponse 
} from '@/services/api';
import bezierService from '@/services/bezierService';

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
  const createPathFromPosesMutation = useMutation<PathResponse, Error, CreatePathFromPosesRequest>({
    mutationFn: async (data: CreatePathFromPosesRequest) => {
      try {
        // Use local bezierService instead of API
        const response = bezierService.createPathFromPoses(data);
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


